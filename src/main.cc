#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "includes/context.hh"
#include "includes/scope_timer.hh"
#include "includes/stats.hh"

using cost = int;
using client = int;
using vehicle = int;
using capacity = int;

using pending_clients = std::unordered_set<client>;
using outsourced_clients = std::vector<client>;
using vehicle_capacities = std::vector<capacity>;
using routes = std::vector<std::vector<client>>;

static bool s_debug{};

constexpr int kDepot{0};  // Depósito é o "cliente" 0.
constexpr int kNotFound{-1};

apa::stats greedy(const apa::context& context);

client greedy_next_client(const apa::context& context, const pending_clients& pending_clients, client origin,
                          capacity vehicle_capacity);

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution);

apa::stats move_client_intra_route(const apa::context& context, const apa::stats& solution);

apa::stats move_client_inter_route(const apa::context& context, const apa::stats& solution);

apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& solution);

int vehicle_load(const apa::context& context, const std::vector<client>& route);

client find_last_client_in_route(const routes& routes, vehicle vehicle);

vehicle find_vehicle_with_most_capacity(const vehicle_capacities& vehicle_capacities);

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

  // Acho que o parser está OK. Qualquer coisa, descomenta aqui e verifica se a instância foi lida corretamente.
  //  if (s_debug) {
  //    std ::cout << context << std::endl;
  //  }

  apa::stats greedy_stats{};
  apa::stats vnd_stats{};

  {
    apa::scope_timer timer{"greedy"};
    greedy_stats = greedy(context);
  }

  {
    apa::scope_timer timer{"vnd"};
    vnd_stats = variable_neighborhood_descent(context, greedy_stats);
  }

  apa::stats_serializer::serialize(greedy_stats, filename + std::string("_greedy.txt"));
  apa::stats_serializer::serialize(vnd_stats, filename + std::string("_vnd.txt"));

  return EXIT_SUCCESS;
}

apa::stats greedy(const apa::context& context) {
  int total_routing_cost{};
  int total_outsourcing_cost{};
  routes routes{};
  pending_clients pending_clients{};
  outsourced_clients outsourced_clients{};
  vehicle_capacities vehicle_capacities{};

  // Inicializa o conjunto de clientes pendentes.
  for (client client = context.clients; client >= 1; client--) {
    pending_clients.insert(client);
  }

  // Inicializa as rotas dos veículos.
  routes = std::vector<std::vector<vehicle>>(context.vehicles, std::vector<client>{});

  // Inicializa as capacidades dos veículos.
  vehicle_capacities = std::vector<vehicle>(context.vehicles, context.vehicle_capacity);

  while (!pending_clients.empty()) {
    const vehicle vehicle{find_vehicle_with_most_capacity(vehicle_capacities)};
    const capacity vehicle_capacity{vehicle_capacities[vehicle]};

    // Considera o último cliente atendido na rota do veículo atual como origem para o próximo cliente a ser atendido.
    // Se a rota estiver vazia, o cliente é o depósito (kDepot).
    const client origin_client{find_last_client_in_route(routes, vehicle)};

    // Encontra o próximo cliente a ser atendido.
    const client target_client{greedy_next_client(context, pending_clients, origin_client, vehicle_capacity)};

    // Cliente encontrado, atende-o.
    if (target_client != kNotFound) {
      if (s_debug) {
        std::cout << "greedy: vehicle " << vehicle << " will serve client " << target_client << " with cost "
                  << context.distance(origin_client, target_client) << std::endl;
      }

      routes[vehicle].push_back(target_client);  // Adiciona o cliente à rota do veículo atual.

      total_routing_cost += context.distance(origin_client, target_client);  // Atualiza o custo de roteamento.
      vehicle_capacities[vehicle] -= context.demand(target_client);          // Atualiza a capacidade do veículo.

      pending_clients.erase(target_client);  // Remove o cliente do conjunto de clientes pendentes.
    }
    // Cliente não pode ser atendido pelo veículo de maior capacidade, então ele deve ser terceirizado.
    else {
      // Neste ponto, nenhum cliente pendente pode ser atendido (mesmo pelo veículo de maior capacidade), então todos os
      // clientes pendentes devem ser terceirizados. Tanto faz qual cliente será terceirizado, então escolhe-se o
      // primeiro cliente pendente.
      const client outsource_client{*pending_clients.begin()};

      total_outsourcing_cost += context.outsourcing_cost(outsource_client);  // Atualiza o custo de terceirização.

      outsourced_clients.push_back(outsource_client);  // Adiciona o cliente à lista de clientes terceirizados.
      pending_clients.erase(outsource_client);         // Remove o cliente do conjunto de clientes pendentes.
    }
  }

  // Neste ponto, todos os clientes foram atendidos ou terceirizados. Agora, é necessário retornar ao depósito.
  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    // O veículo atual não foi utilizado (não possui rota), então não é necessário retornar ao depósito.
    if (routes[vehicle].empty()) {
      continue;
    }

    if (s_debug) {
      std::cout << "greedy: vehicle " << vehicle << " will return to depot with cost "
                << context.distance(routes[vehicle].back(), kDepot) << std::endl;
    }

    // Atualiza o custo de roteamento com o retorno ao depósito.
    total_routing_cost += context.distance(routes[vehicle].back(), kDepot);
  }

  int total_vehicle_cost{};

  // Calcula o custo de utilização dos veículos.
  for (const capacity capacity : vehicle_capacities) {
    if (capacity < context.vehicle_capacity) {
      total_vehicle_cost += context.vehicle_cost;
    }
  }

  const int total_cost{total_routing_cost + total_outsourcing_cost + total_vehicle_cost};

  return {
      total_cost,              // total_cost
      total_routing_cost,      // routing_cost
      total_vehicle_cost,      // vehicles_cost
      total_outsourcing_cost,  // outsourcing_cost
      outsourced_clients,      // outsourced_clients
      routes                   // routes
  };
}

client greedy_next_client(const apa::context& context, const pending_clients& pending_clients, client origin,
                          capacity vehicle_capacity) {
  client client{-1};                              // Inicializa o cliente com -1 (não há cliente possível).
  int min_cost{std::numeric_limits<int>::max()};  // Inicializa o custo com o maior valor possível.

  for (const auto target_client : pending_clients) {
    const int target_client_demand{context.demand(target_client)};
    const int target_client_distance{context.distance(origin, target_client)};

    // Considera o custo da distância.
    int target_client_cost{target_client_distance};

    if (target_client_cost < min_cost && target_client_demand <= vehicle_capacity) {
      client = target_client;
      min_cost = target_client_cost;
    }
  }

  return client;
}

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Estruturas de vizinhança.
  std::vector<apa::stats (*)(const apa::context&, const apa::stats&)> neighborhoods = {
      move_client_intra_route, move_client_inter_route, move_client_with_outsourcing};

  std::size_t neighborhood_index{};
  while (neighborhood_index < neighborhoods.size()) {
    const auto& neighborhood{neighborhoods[neighborhood_index]};

    // Aplica a estrutura de vizinhança na melhor solução encontrada até o momento.
    apa::stats current_solution{neighborhood(context, best_solution)};

    // Avalia a solução atual.
    if (current_solution.total_cost < best_solution.total_cost) {
      best_solution = current_solution;

      // Reseta o índice da estrutura de vizinhança para que todas as estruturas de vizinhança sejam aplicadas na
      // nova melhor solução.
      neighborhood_index = 0;
    } else {
      if (s_debug) {
        std::cout << "vnd: no improvement in "
                  << (neighborhood_index == 0   ? "intra-route"
                      : neighborhood_index == 1 ? "inter-route"
                                                : "outsourcing")
                  << " neighborhood" << std::endl;
      }

      // Incrementa o índice da estrutura de vizinhança. Se o índice igualar ao tamanho do vetor de estruturas de
      // vizinhança, não há mais melhorias possíveis (todas as estruturas de vizinhança foram aplicadas). Neste
      // caso, o algoritmo termina com a melhor solução encontrada.
      neighborhood_index++;
    }
  }

  if (s_debug) {
    std::cout << "vnd: total cost gain: " << best_solution.total_cost - solution.total_cost << std::endl;
  }

  return best_solution;
}

apa::stats move_client_intra_route(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution = solution;
  apa::stats curr_solution = best_solution;

  client* best_lhs_client{};
  client* best_rhs_client{};

  int curr_total_cost{curr_solution.total_cost};
  int best_total_cost{curr_total_cost};

  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    auto& route{curr_solution.routes[vehicle]};

    for (std::size_t lhs_client = 0; lhs_client < route.size(); lhs_client++) {
      for (std::size_t rhs_client = lhs_client + 1; rhs_client < route.size(); rhs_client++) {
        std::swap(route[lhs_client], route[rhs_client]);
        apa::stats temp_solution = apa::rebuild_stats(context, curr_solution);

        if (temp_solution.total_cost < best_total_cost) {
          best_total_cost = temp_solution.total_cost;
          best_lhs_client = &route[lhs_client];
          best_rhs_client = &route[rhs_client];
        }

        std::swap(route[lhs_client], route[rhs_client]);
      }
    }
  }

  // Não houve melhoria na rota atual.
  if (best_total_cost == curr_total_cost) {
    return best_solution;
  }

  if (s_debug) {
    std::cout << "vnd: swapping clients intra-route " << *best_rhs_client << " and " << *best_lhs_client << " ("
              << best_total_cost - curr_total_cost << ")" << std::endl;
  }

  // Troca os clientes e atualiza a solução atual.
  std::swap(*best_lhs_client, *best_rhs_client);
  curr_solution = apa::rebuild_stats(context, curr_solution);
  best_solution = curr_solution;

  return best_solution;
}

apa::stats move_client_inter_route(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution = solution;

  client* best_lhs_client{nullptr};
  client* best_rhs_client{nullptr};

  apa::stats curr_solution = best_solution;

  int curr_total_cost{curr_solution.total_cost};
  int best_total_cost{curr_total_cost};

  // Itera sobre todos os pares de veículos possíveis.
  for (vehicle lhs_vehicle = 0; lhs_vehicle < context.vehicles; lhs_vehicle++) {
    for (vehicle rhs_vehicle = lhs_vehicle + 1; rhs_vehicle < context.vehicles; rhs_vehicle++) {
      // Obtém as rotas dos veículos.
      std::vector<client>& lhs_route = curr_solution.routes[lhs_vehicle];
      std::vector<client>& rhs_route = curr_solution.routes[rhs_vehicle];

      // Obtém a carga corrente dos veículos (soma das demandas dos clientes na rota).
      const int lhs_vehicle_load{vehicle_load(context, lhs_route)};
      const int rhs_vehicle_load{vehicle_load(context, rhs_route)};

      // Itera sobre todos os pares de clientes possíveis entre as rotas.
      for (client& lhs_client : lhs_route) {
        for (client& rhs_client : rhs_route) {
          // Verifica se a troca respeitará a restrição de capacidade dos veículos.
          if (lhs_vehicle_load - context.demand(lhs_client) + context.demand(rhs_client) > context.vehicle_capacity ||
              rhs_vehicle_load - context.demand(rhs_client) + context.demand(lhs_client) > context.vehicle_capacity) {
            continue;
          }

          std::swap(lhs_client, rhs_client);
          apa::stats temp_solution = apa::rebuild_stats(context, curr_solution);

          if (temp_solution.total_cost < best_total_cost) {
            best_total_cost = temp_solution.total_cost;
            best_lhs_client = &lhs_client;
            best_rhs_client = &rhs_client;
          }

          std::swap(lhs_client, rhs_client);
        }
      }
    }
  }

  // Não houve melhoria na troca de clientes entre as rotas.
  if (best_total_cost == curr_total_cost) {
    return best_solution;
  }

  if (s_debug) {
    std::cout << "vnd: swapping clients inter-route " << *best_lhs_client << " and " << *best_rhs_client << " ("
              << best_total_cost - curr_total_cost << ")" << std::endl;
  }

  // Troca os clientes e atualiza a solução atual.
  std::swap(*best_lhs_client, *best_rhs_client);
  curr_solution = apa::rebuild_stats(context, curr_solution);
  best_solution = curr_solution;

  return best_solution;
}

apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& solution) {
  apa::stats current_solution = solution;

  // Verifica se a solução atual já atingiu o limite de terceirização permitido (N - L).
  if (current_solution.outsourced_clients.size() == static_cast<std::size_t>(context.outsourcing_threshold())) {
    return current_solution;
  }

  // Melhor candidato à terceirização.
  std::tuple<client, vehicle, cost> best_candidate{0, 0, std::numeric_limits<cost>::max()};

  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    std::vector<client>& route = current_solution.routes[vehicle];

    for (std::size_t client_index = 0; client_index < route.size(); client_index++) {
      // Vamos calcular o impacto da terceirização do cliente na solução atual.
      int temp_total_cost = current_solution.total_cost;

      // Caso 1: A rota possui um único cliente. [-{d->c1}, -{c1->d}, -{vehicle}].
      if (route.size() == 1) {
        temp_total_cost -= context.distance(kDepot, route[client_index]);
        temp_total_cost -= context.distance(route[client_index], kDepot);
        temp_total_cost -= context.vehicle_cost;
      }
      // Caso geral: A rota possui mais de um cliente.
      else {
        // Caso geral 1: O cliente é o primeiro da rota. [-{d->c1}, -{c1->c2}, +{c2->d}].
        if (client_index == 0) {
          temp_total_cost -= context.distance(kDepot, route[client_index]);
          temp_total_cost -= context.distance(route[client_index], route[client_index + 1]);

          temp_total_cost += context.distance(kDepot, route[client_index + 1]);
        }

        // Caso geral 2: O cliente é o último da rota. [-{c(n-1)->cn}, -{cn->d}, +{c(n-1)->d}].
        if (client_index == route.size() - 1) {
          temp_total_cost -= context.distance(route[client_index - 1], route[client_index]);
          temp_total_cost -= context.distance(route[client_index], kDepot);

          temp_total_cost += context.distance(route[client_index - 1], kDepot);
        }

        // Caso geral 3: O cliente está no meio da rota. [-{c(n-1)->c(n+1)}, -{cn->c(n+1)}, +{c(n-1)->cn}].
        if (client_index > 0 && client_index < route.size() - 1) {
          temp_total_cost -= context.distance(route[client_index - 1], route[client_index]);
          temp_total_cost -= context.distance(route[client_index], route[client_index + 1]);

          temp_total_cost += context.distance(route[client_index - 1], route[client_index + 1]);
        }
      }

      // Adiciona o custo de terceirização do cliente.
      temp_total_cost += context.outsourcing_cost(route[client_index]);

      // É melhor que o candidato atual?
      if (temp_total_cost < std::get<2>(best_candidate)) {
        best_candidate = std::make_tuple(route[client_index], vehicle, temp_total_cost);
      }
    }
  }

  const auto& [best_client, best_vehicle, best_cost] = best_candidate;

  // Verifica se a solução atual é melhor que o candidato.
  if (best_cost >= current_solution.total_cost) {
    return current_solution;
  }

  if (s_debug) {
    std::cout << "vnd: outsourcing client " << best_client << " (" << best_cost - current_solution.total_cost << ")"
              << std::endl;
  }

  // Remove o cliente da rota do veículo.
  current_solution.routes[best_vehicle].erase(std::find(current_solution.routes[best_vehicle].begin(),
                                                        current_solution.routes[best_vehicle].end(), best_client));

  // Adiciona o cliente à lista de clientes terceirizados e atualiza a solução atual.
  current_solution.outsourced_clients.push_back(best_client);
  current_solution = apa::rebuild_stats(context, current_solution);

  if (current_solution.total_cost != best_cost) {
    throw std::runtime_error("O homem nasce bom, errar a simulação de custo total o corrompe.");
  }

  return current_solution;
}

int vehicle_load(const apa::context& context, const std::vector<client>& route) {
  int load{};

  for (const client client : route) {
    load += context.demand(client);
  }

  return load;
}

vehicle find_vehicle_with_most_capacity(const vehicle_capacities& vehicle_capacities) {
  vehicle vehicle{};

  for (std::size_t current_vehicle = 0; current_vehicle < vehicle_capacities.size(); current_vehicle++) {
    if (vehicle_capacities[current_vehicle] > vehicle_capacities[vehicle]) {
      vehicle = static_cast<int>(current_vehicle);
    }
  }

  return vehicle;
}

client find_last_client_in_route(const routes& routes, vehicle vehicle) {
  return routes[vehicle].empty() ? kDepot  // A rota está vazia, então o cliente é o depósito (kDepot).
                                 : routes[vehicle].back();
}