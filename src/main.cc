#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "include/context.hh"
#include "include/gap.hh"
#include "include/stats.hh"
#include "include/stopwatch.hh"

using cost = int;
using client = int;
using demand = int;
using capacity = int;
using distance = int;

static bool s_debug{};

constexpr int kDepot{0};  // Depósito é o "cliente" 0.

template <typename T>
T rnd(T min, T max) {
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<T> dist(min, max);

  return dist(rng);
}

apa::stats greedy(const apa::context& context);

client greedy_next_client(const apa::context& context, const std::unordered_set<client>& pending, client origin,
                          capacity available_capacity);

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution);

apa::stats iterated_local_search(const apa::context& context, const apa::stats& solution,
                                 int iteration_threshold = 1000);

void iterated_local_search_perturbation(const apa::context& context, apa::stats& solution, int threshold);

void move_client_intra_route(const apa::context& context, apa::stats& solution);

void move_client_inter_route(const apa::context& context, apa::stats& solution);

void move_client_with_outsourcing(const apa::context& context, apa::stats& solution);

demand sum_vehicle_demand(const apa::context& context, const std::vector<client>& route);

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.txt> [-d (debug)]" << std::endl;
    return EXIT_FAILURE;
  }

  if (argc == 3 && std::string(argv[2]) == "-d") {
    s_debug = true;
  }

  const std::filesystem::path& input_file{argv[1]};
  const std::string& filename{input_file.stem().string()};

  const auto& context = apa::context_parser::parse(input_file);

  apa::stats greedy_stats{};
  apa::stats vnd_stats{};
  apa::stats ils_stats{};

  {
    apa::stopwatch greedy_sw{"greedy"};
    greedy_stats = greedy(context);
  }

  std::cout << "gap: " << std::fixed << std::setprecision(2) << apa::gap(filename, greedy_stats.total_cost()) << "%"
            << std::endl;

  std::cout << std::endl;

  {
    apa::stopwatch vnd_sw{"vnd"};
    vnd_stats = variable_neighborhood_descent(context, greedy_stats);
  }

  std::cout << "gap: " << std::fixed << std::setprecision(2) << apa::gap(filename, vnd_stats.total_cost()) << "%"
            << std::endl;

  std::cout << std::endl;

  {
    apa::stopwatch ils_sw{"ils"};
    ils_stats = iterated_local_search(context, greedy_stats);
  }

  std::cout << "gap: " << std::fixed << std::setprecision(2) << apa::gap(filename, ils_stats.total_cost()) << "%"
            << std::endl;

  apa::stats_serializer::serialize(greedy_stats, filename + std::string("_greedy.txt"));
  apa::stats_serializer::serialize(vnd_stats, filename + std::string("_vnd.txt"));
  apa::stats_serializer::serialize(ils_stats, filename + std::string("_ils.txt"));
}

apa::stats greedy(const apa::context& context) {
  int routing_cost{};
  int outsourcing_cost{};
  int vehicle_cost{};
  std::vector<std::vector<client>> routes{};
  std::unordered_set<client> pending{};
  std::vector<client> outsourced{};
  std::vector<capacity> capacities{};

  for (client client = context.clients; client >= 1; client--) {
    pending.insert(client);
  }

  routes = std::vector<std::vector<client>>(context.vehicles, std::vector<client>{kDepot});
  capacities = std::vector<capacity>(context.vehicles, context.vehicle_capacity);

  std::size_t current_vehicle{0};

  // Aloca os clientes nas rotas.
  while (!pending.empty()) {
    // Verifica se atingiu o número máximo de veículos.
    if (current_vehicle == routes.size()) {
      break;
    }

    capacity available_capacity{capacities[current_vehicle]};

    // Obtém o cliente de origem. Se a rota estiver vazia, então o cliente de origem é o depósito.
    client origin_client{routes[current_vehicle].size() == 1 ? kDepot : routes[current_vehicle].back()};

    // Obtém o cliente mais próximo do cliente de origem. Considera a capacidade do veículo atual.
    client target_client{greedy_next_client(context, pending, origin_client, available_capacity)};

    if (target_client) {
      int demand{context.demand(target_client)};
      int distance{context.distance(origin_client, target_client)};

      if (s_debug) {
        std::cout << "greedy: vehicle " << current_vehicle << " will serve client " << target_client << " with cost "
                  << distance << std::endl;
      }

      routes[current_vehicle].push_back(target_client);

      capacities[current_vehicle] -= demand;
      routing_cost += distance;

      pending.erase(target_client);

    } else {
      current_vehicle++;
      continue;
    }
  }

  // Terceizia os clientes que não foram alocados em nenhuma rota.
  for (const auto& client : pending) {
    outsourced.push_back(client);
    outsourcing_cost += context.outsourcing_cost(client);
  }

  // Retorna ao depósito e atualiza os custos.
  for (std::size_t vehicle = 0; vehicle < routes.size(); vehicle++) {
    const auto& route{routes[vehicle]};
    if (route.size() == 1) {  // Veículo não saiu do depósito.
      continue;
    }

    if (s_debug) {
      std::cout << "greedy: current_vehicle " << vehicle << " will return to depot with cost "
                << context.distance(routes[vehicle].back(), kDepot) << std::endl;
    }

    vehicle_cost += context.vehicle_cost;
    routing_cost += context.distance(route.back(), kDepot);
    routes[vehicle].push_back(kDepot);
  }

  return {
      routing_cost,      // routing_cost
      vehicle_cost,      // vehicles_cost
      outsourcing_cost,  // outsourcing_cost
      outsourced,        // outsourced
      routes             // routes
  };
}

client greedy_next_client(const apa::context& context, const std::unordered_set<client>& pending, client origin,
                          capacity available_capacity) {
  client next_client{0};
  cost best_cost{std::numeric_limits<int>::max()};

  for (const client client : pending) {
    const demand demand{context.demand(client)};
    const distance distance{context.distance(origin, client)};

    if (distance < best_cost && demand <= available_capacity) {
      next_client = client;
      best_cost = distance;
    }
  }

  return next_client;
}

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Estruturas de vizinhança.
  std::vector<std::pair<std::string, void (*)(const apa::context&, apa::stats&)>> neighborhoods = {
      {"intra-route", move_client_intra_route},
      {"inter-route", move_client_inter_route},
      {"outsourcing", move_client_with_outsourcing}};

  // Indica quantas estruturas de vizinhança foram aplicadas sem melhoria na solução. Se o valor for igual ao número de
  // estruturas de vizinhança, então todas as estruturas de vizinhança foram aplicadas sem melhoria na solução e o VND
  // pode ser encerrado.
  std::size_t current_neighborhood{};

  while (current_neighborhood < neighborhoods.size()) {
    const auto& [neighborhood, neighborhood_function] = neighborhoods[current_neighborhood];

    apa::stats new_solution{best_solution};

    // Aplica a estrutura de vizinhança atual.
    neighborhood_function(context, new_solution);

    // Avalia a solução atual.
    if (new_solution.total_cost() < best_solution.total_cost()) {
      best_solution = new_solution;
      current_neighborhood = 0;  // Reinicia o contador de estruturas de vizinhança.
    } else {
      if (s_debug) {
        std::cout << "vnd: no improvement in " << neighborhood << std::endl;
      }

      current_neighborhood++;
    }
  }

  if (s_debug) {
    std::cout << "vnd: total cost gain: " << best_solution.total_cost() - solution.total_cost() << std::endl;
  }

  return best_solution;
}

apa::stats iterated_local_search(const apa::context& context, const apa::stats& solution, int iteration_threshold) {
  apa::stats best_solution{solution};

  const double time_limit{1000 * 60 * 5};  // 5 minutos

  int iteration{};

  // Limite de perturbação da solução atual.
  int perturbation_threshold{1};

  auto start = std::chrono::steady_clock::now();

  double elapsed_time{};

  // Executa o ILS até atingir o limite de iterações sem melhoria ou o limite de tempo.
  while (iteration < iteration_threshold && elapsed_time < time_limit) {
    apa::stats new_solution{best_solution};

    // Perturba a solução atual.
    iterated_local_search_perturbation(context, new_solution, perturbation_threshold);

    // Aplica o VND na solução perturbada.
    new_solution = variable_neighborhood_descent(context, new_solution);

    // Avalia a nova solução.
    if (new_solution.total_cost() < best_solution.total_cost()) {
      best_solution = new_solution;
      iteration = 0;  // Reinicia o contador de iterações.
    } else {
      iteration++;

      // Incrementa em 5 o limite de perturbação a cada 10% das iterações sem melhoria na solução.
      if (iteration % (iteration_threshold / 10) == 0) {
        perturbation_threshold += 5;
      }

      if (s_debug) {
        std::cout << "ils: iteration " << iteration << " (perturb threshold: " << perturbation_threshold << ")"
                  << std::endl;
      }
    }

    // Atualiza o tempo decorrido.
    elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }

  return best_solution;
}

void iterated_local_search_perturbation(const apa::context& context, apa::stats& solution, int threshold) {
  if (threshold == 0) {
    return;
  }

  apa::stats best_solution{solution};

  std::vector<client>* lhs_route;
  std::vector<client>* rhs_route;

  client* lhs_client;
  client* rhs_client;

  for (int i{}; i < threshold; i++) {
    // Obtem duas rotas aleatorias (podem até ser a mesma rota) que possuem pelo menos 1 cliente cada
    do {
      lhs_route = &best_solution.routes[rnd<std::size_t>(0, best_solution.routes.size() - 1)];
      rhs_route = &best_solution.routes[rnd<std::size_t>(0, best_solution.routes.size() - 1)];
    } while (lhs_route->size() <= 2 || rhs_route->size() <= 2);

    // Obtem dois clientes aleatorios distintos das rotas selecionadas
    lhs_client = &lhs_route->at(rnd<std::size_t>(1, lhs_route->size() - 2));
    rhs_client = &rhs_route->at(rnd<std::size_t>(1, rhs_route->size() - 2));

    if (lhs_client == rhs_client) {
      continue;
    }

    // Troca os clientes (intra/inter-route)
    std::swap(*lhs_client, *rhs_client);
  }

  // Atualiza o custo de roteamento da solução atual.
  // Isso aqui poderia ser removido fazendo a simulação do swap e calculando o custo de roteamento da solução atual.
  // Mas acho que está OK por enquanto. Nem tudo são flores.
  solution = apa::rebuild_stats(context, best_solution);
}

void move_client_intra_route(const apa::context& context, apa::stats& solution) {
  // Melhor candidato à troca de clientes.
  std::tuple<client*, client*, cost> best_move{
      nullptr,               // best_lhs_client
      nullptr,               // best_rhs_client
      solution.routing_cost  // best_routing_cost
  };

  for (std::size_t vehicle{}; vehicle < solution.routes.size(); vehicle++) {
    auto& route{solution.routes[vehicle]};

    for (std::size_t lhs_client{1}; lhs_client < route.size() - 1; lhs_client++) {
      for (std::size_t rhs_client{lhs_client + 1}; rhs_client < route.size() - 1; rhs_client++) {
        int new_routing_cost{solution.routing_cost};

        // .. lhs_client0 -> lhs_client1 -> lhs_client2 ..

        new_routing_cost -= context.distance(route[lhs_client - 1], route[lhs_client]);
        new_routing_cost -= context.distance(route[rhs_client], route[rhs_client + 1]);

        new_routing_cost += context.distance(route[lhs_client - 1], route[rhs_client]);
        new_routing_cost += context.distance(route[lhs_client], route[rhs_client + 1]);

        bool adjacent{lhs_client + 1 == rhs_client};
        if (adjacent) {
          new_routing_cost -= context.distance(route[lhs_client], route[rhs_client]);

          new_routing_cost += context.distance(route[rhs_client], route[lhs_client]);
        } else {
          new_routing_cost -= context.distance(route[lhs_client], route[lhs_client + 1]);
          new_routing_cost -= context.distance(route[rhs_client - 1], route[rhs_client]);

          new_routing_cost += context.distance(route[rhs_client], route[lhs_client + 1]);
          new_routing_cost += context.distance(route[rhs_client - 1], route[lhs_client]);
        }

        // É melhor que o candidato atual?
        if (new_routing_cost < std::get<2>(best_move)) {
          best_move = std::make_tuple(&route[lhs_client], &route[rhs_client], new_routing_cost);
        }
      }
    }
  }

  auto& [best_lhs_client, best_rhs_client, best_routing_cost] = best_move;

  // Verifica se o custo de roteamento da solução atual é melhor que o candidato a melhor solução.
  if (best_routing_cost >= solution.routing_cost) {
    return;
  }

  if (s_debug) {
    int total_cost_gain{solution.total_cost() - solution.routing_cost + best_routing_cost - solution.total_cost()};

    std::cout << "vnd: swapping clients intra-route " << *best_lhs_client << " and " << *best_rhs_client << " ("
              << total_cost_gain << ")" << std::endl;
  }

  // Troca os clientes
  std::swap(*best_lhs_client, *best_rhs_client);

  // Atualiza o custo de roteamento da solução atual.
  solution.routing_cost = best_routing_cost;
}

void move_client_inter_route(const apa::context& context, apa::stats& solution) {
  // Melhor candidato à troca de clientes.
  std::tuple<client*, client*, cost> best_move{
      nullptr,               // best_lhs_client
      nullptr,               // best_rhs_client
      solution.routing_cost  // best_routing_cost
  };

  // Itera sobre todos os pares de veículos possíveis.
  for (std::size_t lhs_vehicle{}; lhs_vehicle < solution.routes.size(); lhs_vehicle++) {
    std::vector<client>& lhs_route = solution.routes[lhs_vehicle];

    for (std::size_t rhs_vehicle{lhs_vehicle + 1}; rhs_vehicle < solution.routes.size(); rhs_vehicle++) {
      std::vector<client>& rhs_route = solution.routes[rhs_vehicle];

      // Obtém a carga total dos veículos.
      const demand total_lhs_vehicle_demand{sum_vehicle_demand(context, lhs_route)};
      const demand total_rhs_vehicle_demand{sum_vehicle_demand(context, rhs_route)};

      for (std::size_t lhs_client{1}; lhs_client < lhs_route.size() - 1; lhs_client++) {
        for (std::size_t rhs_client{lhs_client + 1}; rhs_client < rhs_route.size() - 1; rhs_client++) {
          // Carga total do veículo lhs após a troca.
          demand new_lhs_vehicle_demand{total_lhs_vehicle_demand - context.demand(lhs_route[lhs_client]) +
                                        context.demand(rhs_route[rhs_client])};

          // Carga total do veículo rhs após a troca.
          demand new_rhs_vehicle_demand{total_rhs_vehicle_demand - context.demand(rhs_route[rhs_client]) +
                                        context.demand(lhs_route[lhs_client])};

          // Verifica se a troca respeitará a restrição de capacidade dos veículos.
          if (new_lhs_vehicle_demand > context.vehicle_capacity || new_rhs_vehicle_demand > context.vehicle_capacity) {
            continue;
          }

          cost new_routing_cost{solution.routing_cost};

          // .. lhs_client0 -> lhs_client1 -> lhs_client2 ..
          // .. rhs_client0 -> rhs_client1 -> rhs_client2 ..
          //
          // swap(lhs_client1, rhs_client1)
          //
          // .. lhs_client0 -> rhs_client1 -> lhs_client2 ..
          // .. rhs_client0 -> lhs_client1 -> rhs_client2 ..

          new_routing_cost -= context.distance(lhs_route[lhs_client - 1], lhs_route[lhs_client]);
          new_routing_cost -= context.distance(lhs_route[lhs_client], lhs_route[lhs_client + 1]);
          new_routing_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
          new_routing_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

          new_routing_cost += context.distance(lhs_route[lhs_client - 1], rhs_route[rhs_client]);
          new_routing_cost += context.distance(rhs_route[rhs_client], lhs_route[lhs_client + 1]);
          new_routing_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
          new_routing_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);

          // É melhor que o candidato atual?
          if (new_routing_cost < std::get<2>(best_move)) {
            best_move = std::make_tuple(&lhs_route[lhs_client], &rhs_route[rhs_client], new_routing_cost);
          }
        }
      }
    }
  }

  auto& [best_lhs_client, best_rhs_client, best_routing_cost] = best_move;

  // Verifica se o custo de roteamento da solução atual é melhor que o candidato a melhor solução.
  if (best_routing_cost >= solution.routing_cost) {
    return;
  }

  if (s_debug) {
    int total_cost_gain{solution.total_cost() - solution.routing_cost + best_routing_cost - solution.total_cost()};

    std::cout << "vnd: swapping clients inter-route " << *best_lhs_client << " and " << *best_rhs_client << " ("
              << total_cost_gain << ")" << std::endl;
  }

  // Troca os clientes
  std::swap(*best_lhs_client, *best_rhs_client);

  // Atualiza o custo de roteamento da solução atual.
  solution.routing_cost = best_routing_cost;
}

void move_client_with_outsourcing(const apa::context& context, apa::stats& solution) {
  // Verifica se a solução atual já atingiu o limite de terceirização permitido (N - L).
  if (solution.outsourced.size() == static_cast<std::size_t>(context.outsourcing_threshold())) {
    return;
  }

  // Melhor candidato à terceirização.
  std::tuple<client, std::size_t, cost, cost, cost, cost> best_move{0,                          // best_client
                                                                    0,                          // best_vehicle
                                                                    solution.routing_cost,      // best_routing_cost
                                                                    solution.outsourcing_cost,  // best_outsourcing_cost
                                                                    solution.vehicles_cost,     // best_vehicles_cost
                                                                    solution.total_cost()};     // best_total_cost

  for (std::size_t vehicle{}; vehicle < solution.routes.size(); vehicle++) {
    std::vector<client>& route{solution.routes[vehicle]};

    for (std::size_t client{1}; client < route.size() - 1; client++) {
      cost new_routing_cost{solution.routing_cost};
      cost new_outsourcing_cost{solution.outsourcing_cost};
      cost new_vehicles_cost{solution.vehicles_cost};

      // .. client0 -> client1 -> client2 ..

      new_routing_cost -= context.distance(route[client - 1], route[client]);
      new_routing_cost -= context.distance(route[client], route[client + 1]);

      new_routing_cost += context.distance(route[client - 1], route[client + 1]);

      // Adiciona o custo de terceirização do cliente.
      new_outsourcing_cost += context.outsourcing_cost(route[client]);

      // Remove o custo de utilização do veículo se a rota ficar sem clientes.
      if (route.size() == 3) {
        new_vehicles_cost -= context.vehicle_cost;
      }

      // É melhor que o candidato atual?
      cost new_total_cost{new_routing_cost + new_outsourcing_cost + new_vehicles_cost};
      cost best_total_cost{std::get<2>(best_move) + std::get<3>(best_move) + std::get<4>(best_move)};

      if (new_total_cost < best_total_cost) {
        best_move = std::make_tuple(route[client], vehicle, new_routing_cost, new_outsourcing_cost, new_vehicles_cost,
                                    new_total_cost);
      }
    }
  }

  const auto& [best_client, best_vehicle, best_routing_cost, best_outsourcing_cost, best_vehicles_cost,
               best_total_cost] = best_move;

  // Verifica se a solução atual é melhor que o candidato.
  if (best_total_cost >= solution.total_cost()) {
    return;
  }

  if (s_debug) {
    std::cout << "vnd: outsourcing client " << best_client << " (" << best_total_cost - solution.total_cost() << ")"
              << std::endl;
  }

  // Remove o cliente da rota do veículo.
  auto best_client_it{
      std::find(solution.routes[best_vehicle].begin(), solution.routes[best_vehicle].end(), best_client)};
  solution.routes[best_vehicle].erase(best_client_it);

  // Adiciona o cliente à lista de clientes terceirizados
  solution.outsourced.push_back(best_client);

  // Atualiza os custos de roteamento, terceirização e utilização dos veículos da solução atual.
  solution.routing_cost = best_routing_cost;
  solution.outsourcing_cost = best_outsourcing_cost;
  solution.vehicles_cost = best_vehicles_cost;
}

demand sum_vehicle_demand(const apa::context& context, const std::vector<client>& route) {
  demand demand{};

  for (std::size_t client_index{1}; client_index < route.size() - 1; client_index++) {
    demand += context.demand(route[client_index]);
  }

  return demand;
}