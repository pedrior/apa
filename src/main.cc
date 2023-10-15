#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "includes/context.hh"
#include "includes/stats.hh"
#include "includes/stopwatch.hh"

using cost = int;
using client = int;
using vehicle = int;
using capacity = int;

static bool s_debug{};

constexpr int kDepot{0};  // Depósito é o "cliente" 0.
constexpr int kNotFound{-1};

apa::stats greedy(const apa::context& context);

client greedy_next_client(const apa::context& context, const std::unordered_set<client>& pending, client origin,
                          capacity capacity);

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution);

apa::stats move_client_intra_route(const apa::context& context, const apa::stats& solution);

apa::stats move_client_inter_route(const apa::context& context, const apa::stats& solution);

apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& solution);

int sum_vehicle_load(const apa::context& context, const std::vector<client>& route);

client find_last_client(const std::vector<std::vector<client>>& routes, vehicle vehicle);

vehicle find_best_vehicle(const std::vector<capacity>& capacities);

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

  {
    apa::stopwatch greedy_sw{"greedy"};
    greedy_stats = greedy(context);
  }

  {
    apa::stopwatch vnd_sw{"vnd"};
    vnd_stats = variable_neighborhood_descent(context, greedy_stats);
  }

  apa::stats_serializer::serialize(greedy_stats, filename + std::string("_greedy.txt"));
  apa::stats_serializer::serialize(vnd_stats, filename + std::string("_vnd.txt"));
}

apa::stats greedy(const apa::context& context) {
  int routing_cost{};
  int outsourcing_cost{};
  std::vector<std::vector<client>> routes{};
  std::unordered_set<client> pending{};
  std::vector<client> outsourced{};
  std::vector<capacity> capacities{};

  // Inicializa o conjunto de clientes pendentes.
  for (client client = context.clients; client >= 1; client--) {
    pending.insert(client);
  }

  // Inicializa as rotas dos veículos.
  routes = std::vector<std::vector<vehicle>>(context.vehicles, std::vector<client>{});

  // Inicializa as capacidades dos veículos.
  capacities = std::vector<vehicle>(context.vehicles, context.vehicle_capacity);

  while (!pending.empty()) {
    const vehicle vehicle{find_best_vehicle(capacities)};
    const capacity vehicle_capacity{capacities[vehicle]};

    // Considera o último cliente atendido na rota do veículo atual como origem para o próximo cliente a ser atendido.
    // Se a rota estiver vazia, o cliente é o depósito (kDepot).
    const client origin_client{find_last_client(routes, vehicle)};

    // Encontra o próximo cliente a ser atendido.
    const client target_client{greedy_next_client(context, pending, origin_client, vehicle_capacity)};

    // Cliente encontrado, atende-o.
    if (target_client != kNotFound) {
      if (s_debug) {
        std::cout << "greedy: vehicle " << vehicle << " will serve client " << target_client << " with cost "
                  << context.distance(origin_client, target_client) << std::endl;
      }

      routes[vehicle].push_back(target_client);  // Adiciona o cliente à rota do veículo atual.

      routing_cost += context.distance(origin_client, target_client);  // Atualiza o custo de roteamento.
      capacities[vehicle] -= context.demand(target_client);            // Atualiza a capacidade do veículo.

      pending.erase(target_client);  // Remove o cliente do conjunto de clientes pendentes.
    }
    // Cliente não pode ser atendido pelo veículo de maior capacidade, então ele deve ser terceirizado.
    else {
      // Neste ponto, nenhum cliente pendente pode ser atendido (mesmo pelo veículo de maior capacidade), então todos os
      // clientes pendentes devem ser terceirizados. Tanto faz qual cliente será terceirizado, então escolhe-se o
      // primeiro cliente pendente.
      const client outsource_client{*pending.begin()};

      outsourcing_cost += context.outsourcing_cost(outsource_client);  // Atualiza o custo de terceirização.

      outsourced.push_back(outsource_client);  // Adiciona o cliente à lista de clientes terceirizados.
      pending.erase(outsource_client);         // Remove o cliente do conjunto de clientes pendentes.
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
    routing_cost += context.distance(routes[vehicle].back(), kDepot);
  }

  int total_vehicle_cost{};

  // Calcula o custo de utilização dos veículos.
  for (const capacity capacity : capacities) {
    if (capacity < context.vehicle_capacity) {
      total_vehicle_cost += context.vehicle_cost;
    }
  }

  const int total_cost{routing_cost + outsourcing_cost + total_vehicle_cost};

  return {
      total_cost,          // total_cost
      routing_cost,        // routing_cost
      total_vehicle_cost,  // vehicles_cost
      outsourcing_cost,    // outsourcing_cost
      outsourced,          // outsourced
      routes               // routes
  };
}

client greedy_next_client(const apa::context& context, const std::unordered_set<client>& pending, client origin,
                          capacity capacity) {
  client client{-1};                              // Inicializa o cliente com -1 (não há cliente possível).
  int min_cost{std::numeric_limits<int>::max()};  // Inicializa o custo com o maior valor possível.

  for (const auto target_client : pending) {
    const int target_client_demand{context.demand(target_client)};
    const int target_client_distance{context.distance(origin, target_client)};

    // Considera o custo da distância.
    int target_client_cost{target_client_distance};

    if (target_client_cost < min_cost && target_client_demand <= capacity) {
      client = target_client;
      min_cost = target_client_cost;
    }
  }

  return client;
}

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Estruturas de vizinhança.
  std::vector<std::pair<std::string, apa::stats (*)(const apa::context&, const apa::stats&)>> neighborhoods = {
      {"intra-route", move_client_intra_route},
      {"inter-route", move_client_inter_route},
      {"outsourcing", move_client_with_outsourcing}};

  // Indica quantas estruturas de vizinhança foram aplicadas sem melhoria na solução. Se o valor for igual ao número de
  // estruturas de vizinhança, então todas as estruturas de vizinhança foram aplicadas sem melhoria na solução e o VND
  // pode ser encerrado.
  std::size_t current_neighborhood{};

  while (current_neighborhood < neighborhoods.size()) {
    const auto& [neighborhood, neighborhood_function] = neighborhoods[current_neighborhood];

    // Aplica a estrutura de vizinhança na melhor solução encontrada até o momento.
    apa::stats new_solution{neighborhood_function(context, best_solution)};

    // Avalia a solução atual.
    if (new_solution.total_cost < best_solution.total_cost) {
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
    std::cout << "vnd: total cost gain: " << best_solution.total_cost - solution.total_cost << std::endl;
  }

  return best_solution;
}

apa::stats move_client_intra_route(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Melhor candidato à troca de clientes.
  std::tuple<client*, client*, cost> best_move{nullptr, nullptr, std::numeric_limits<cost>::max()};

  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    auto& route{best_solution.routes[vehicle]};

    // Não há clientes suficientes para troca.
    if (route.size() < 2) {
      continue;
    }

    for (std::size_t lhs_client = 0; lhs_client < route.size(); lhs_client++) {
      for (std::size_t rhs_client = lhs_client + 1; rhs_client < route.size(); rhs_client++) {
        int new_total_cost{best_solution.total_cost};
        bool adjacent_clients{lhs_client + 1 == rhs_client};

        // Caso 1: clientes adjacentes.
        if (adjacent_clients) {
          // Caso 1.1: A rota possui apenas um par clientes.
          if (route.size() == 2) {
            new_total_cost -= context.distance(kDepot, route[lhs_client]);
            new_total_cost -= context.distance(route[lhs_client], route[rhs_client]);
            new_total_cost -= context.distance(route[rhs_client], kDepot);

            new_total_cost += context.distance(kDepot, route[rhs_client]);
            new_total_cost += context.distance(route[rhs_client], route[lhs_client]);
            new_total_cost += context.distance(route[lhs_client], kDepot);
          } else {
            // Caso 1.2 (l, r, ...): O par de cliente está no início da rota.
            if (lhs_client == 0 && rhs_client == lhs_client + 1) {
              new_total_cost -= context.distance(kDepot, route[lhs_client]);
              new_total_cost -= context.distance(route[lhs_client], route[rhs_client]);
              new_total_cost -= context.distance(route[rhs_client], route[rhs_client + 1]);

              new_total_cost += context.distance(kDepot, route[rhs_client]);
              new_total_cost += context.distance(route[rhs_client], route[lhs_client]);
              new_total_cost += context.distance(route[lhs_client], route[rhs_client + 1]);
            }
            // Caso 1.3 (..., l, r): O par de cliente está no final da rota.
            else if (lhs_client == route.size() - 2 && rhs_client == lhs_client + 1) {
              new_total_cost -= context.distance(route[lhs_client - 1], route[lhs_client]);
              new_total_cost -= context.distance(route[lhs_client], route[rhs_client]);
              new_total_cost -= context.distance(route[rhs_client], kDepot);

              new_total_cost += context.distance(route[lhs_client - 1], route[rhs_client]);
              new_total_cost += context.distance(route[rhs_client], route[lhs_client]);
              new_total_cost += context.distance(route[lhs_client], kDepot);
            }
            // Caso 1.4 (..., l, r, ...): O par de cliente está no meio da rota.
            else if (lhs_client > 0 && rhs_client < route.size() - 1) {
              new_total_cost -= context.distance(route[lhs_client - 1], route[lhs_client]);
              new_total_cost -= context.distance(route[lhs_client], route[rhs_client]);
              new_total_cost -= context.distance(route[rhs_client], route[rhs_client + 1]);

              new_total_cost += context.distance(route[lhs_client - 1], route[rhs_client]);
              new_total_cost += context.distance(route[rhs_client], route[lhs_client]);
              new_total_cost += context.distance(route[lhs_client], route[rhs_client + 1]);
            } else {
              throw std::runtime_error("[intra-route] Caso 1.5: não tratado.");
            }
          }
        }
        // Caso 2: Clientes não adjacentes.
        else {
          // Caso 2.1 (l, ..., r, ...): O par não adjacentes está no início da rota.
          if (lhs_client == 0 && rhs_client > lhs_client + 1 && rhs_client < route.size() - 1) {
            new_total_cost -= context.distance(kDepot, route[lhs_client]);
            new_total_cost -= context.distance(route[lhs_client], route[lhs_client + 1]);
            new_total_cost -= context.distance(route[rhs_client - 1], route[rhs_client]);
            new_total_cost -= context.distance(route[rhs_client], route[rhs_client + 1]);

            new_total_cost += context.distance(kDepot, route[rhs_client]);
            new_total_cost += context.distance(route[rhs_client], route[lhs_client + 1]);
            new_total_cost += context.distance(route[rhs_client - 1], route[lhs_client]);
            new_total_cost += context.distance(route[lhs_client], route[rhs_client + 1]);
          }
          // Caso 2.2 (..., l, ..., r, ...): O par não adjacentes está no meio da rota.
          else if (lhs_client > 0 && rhs_client > lhs_client + 1 && rhs_client < route.size() - 1) {
            new_total_cost -= context.distance(route[lhs_client - 1], route[lhs_client]);
            new_total_cost -= context.distance(route[lhs_client], route[lhs_client + 1]);
            new_total_cost -= context.distance(route[rhs_client - 1], route[rhs_client]);
            new_total_cost -= context.distance(route[rhs_client], route[rhs_client + 1]);

            new_total_cost += context.distance(route[lhs_client - 1], route[rhs_client]);
            new_total_cost += context.distance(route[rhs_client], route[lhs_client + 1]);
            new_total_cost += context.distance(route[rhs_client - 1], route[lhs_client]);
            new_total_cost += context.distance(route[lhs_client], route[rhs_client + 1]);
          }
          // Caso 2.3 (..., l, ..., r): O par não adjacentes está no final da rota.
          else if (lhs_client > 0 && rhs_client > lhs_client + 1 && rhs_client == route.size() - 1) {
            new_total_cost -= context.distance(route[lhs_client - 1], route[lhs_client]);
            new_total_cost -= context.distance(route[lhs_client], route[lhs_client + 1]);
            new_total_cost -= context.distance(route[rhs_client - 1], route[rhs_client]);
            new_total_cost -= context.distance(route[rhs_client], kDepot);

            new_total_cost += context.distance(route[lhs_client - 1], route[rhs_client]);
            new_total_cost += context.distance(route[rhs_client], route[lhs_client + 1]);
            new_total_cost += context.distance(route[rhs_client - 1], route[lhs_client]);
            new_total_cost += context.distance(route[lhs_client], kDepot);
          }
          // Caso 2.4 (l, ..., r): O par não adjacentes é o primeiro e o último da rota.
          else if (lhs_client == 0 && rhs_client == route.size() - 1) {
            new_total_cost -= context.distance(kDepot, route[lhs_client]);
            new_total_cost -= context.distance(route[lhs_client], route[lhs_client + 1]);
            new_total_cost -= context.distance(route[rhs_client - 1], route[rhs_client]);
            new_total_cost -= context.distance(route[rhs_client], kDepot);

            new_total_cost += context.distance(kDepot, route[rhs_client]);
            new_total_cost += context.distance(route[rhs_client], route[lhs_client + 1]);
            new_total_cost += context.distance(route[rhs_client - 1], route[lhs_client]);
            new_total_cost += context.distance(route[lhs_client], kDepot);
          } else {
            throw std::runtime_error("[intra-route] Caso 2.5: não tratado.");
          }
        }

        // É melhor que o candidato atual?
        if (new_total_cost < std::get<2>(best_move)) {
          best_move = std::make_tuple(&route[lhs_client], &route[rhs_client], new_total_cost);
        }
      }
    }
  }

  auto& [best_lhs_client, best_rhs_client, best_cost] = best_move;

  // Verifica se a solução atual é melhor que o candidato.
  if (best_cost >= best_solution.total_cost) {
    return best_solution;
  }

  if (s_debug) {
    std::cout << "vnd: swapping clients intra-route " << *best_lhs_client << " and " << *best_rhs_client << " ("
              << best_cost - best_solution.total_cost << ")" << std::endl;
  }

  // Troca os clientes e atualiza a solução atual.
  std::swap(*best_lhs_client, *best_rhs_client);
  best_solution = apa::rebuild_stats(context, best_solution);

  if (best_solution.total_cost != best_cost) {
    std::cout << "best_cost: " << best_cost << " best_solution.total_cost: " << best_solution.total_cost << std::endl;
    throw std::runtime_error("[intra-route] best_cost != best_solution.total_cost");
  }

  return best_solution;
}

apa::stats move_client_inter_route(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Melhor candidato à troca de clientes.
  std::tuple<client*, client*, cost> best_move{nullptr, nullptr, std::numeric_limits<cost>::max()};

  // Itera sobre todos os pares de veículos possíveis.
  for (vehicle lhs_vehicle = 0; lhs_vehicle < context.vehicles; lhs_vehicle++) {
    std::vector<client>& lhs_route = best_solution.routes[lhs_vehicle];
    if (lhs_route.empty()) {
      continue;
    }

    for (vehicle rhs_vehicle = lhs_vehicle + 1; rhs_vehicle < context.vehicles; rhs_vehicle++) {
      std::vector<client>& rhs_route = best_solution.routes[rhs_vehicle];
      if (rhs_route.empty()) {
        continue;
      }

      // Obtém a carga corrente dos veículos (soma das demandas dos clientes).
      const int lhs_vehicle_load{sum_vehicle_load(context, lhs_route)};
      const int rhs_vehicle_load{sum_vehicle_load(context, rhs_route)};

      for (std::size_t lhs_client = 0; lhs_client < lhs_route.size(); lhs_client++) {
        for (std::size_t rhs_client = lhs_client + 1; rhs_client < rhs_route.size(); rhs_client++) {
          int new_lhs_vehicle_load{lhs_vehicle_load - context.demand(lhs_route[lhs_client]) +
                                   context.demand(rhs_route[rhs_client])};
          int new_rhs_vehicle_load{rhs_vehicle_load - context.demand(rhs_route[rhs_client]) +
                                   context.demand(lhs_route[lhs_client])};

          // Verifica se a troca respeitará a restrição de capacidade dos veículos.
          if (new_lhs_vehicle_load > context.vehicle_capacity || new_rhs_vehicle_load > context.vehicle_capacity) {
            continue;
          }

          cost new_total_cost{best_solution.total_cost};

          // Caso 1: A rota lhs possui apenas um cliente.
          if (lhs_route.size() == 1) {
            // Caso 1.1: A rota rhs possui apenas um cliente.
            if (rhs_route.size() == 1) {
              continue;  // A troca não irá alterar o custo total.
            }

            // Caso 1.2: A rota rhs possui mais de um cliente e o cliente é o primeiro da rota.
            else if (rhs_route.size() > 1 && rhs_client == 0) {
              new_total_cost -= context.distance(lhs_route[lhs_client], kDepot);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(rhs_route[rhs_client], kDepot);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            }

            // Caso 1.3: A rota rhs possui mais de um cliente e o cliente é o último da rota.
            else if (rhs_route.size() > 1 && rhs_client == rhs_route.size() - 1) {
              new_total_cost -= context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);

              new_total_cost += context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
            }

            // Caso 1.4: A rota rhs possui mais de um cliente e o cliente está no meio da rota.
            else if (rhs_route.size() > 1 && rhs_client > 0 && rhs_client < rhs_route.size() - 1) {
              new_total_cost -= context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost -= context.distance(lhs_route[lhs_client], kDepot);
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost += context.distance(rhs_route[rhs_client], kDepot);
              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            } else {
              throw std::runtime_error("[inter-route] Caso 1.5: não tratado.");
            }
          }

          // Caso 2: A rota lhs possui mais de um cliente e o cliente é o primeiro da rota.
          else if (lhs_route.size() > 1 && lhs_client == 0) {
            new_total_cost -= context.distance(lhs_route[lhs_client], lhs_route[lhs_client + 1]);

            new_total_cost += context.distance(rhs_route[rhs_client], lhs_route[lhs_client + 1]);

            // Caso 2.1: A rota rhs possui apenas um cliente.
            if (rhs_route.size() == 1) {
              new_total_cost -= context.distance(rhs_route[rhs_client], kDepot);

              new_total_cost += context.distance(lhs_route[lhs_client], kDepot);
            }

            // Caso 2.2: A rota rhs possui mais de um cliente e o cliente é o primeiro da rota.
            else if (rhs_route.size() > 1 && rhs_client == 0) {
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            }

            // Caso 2.3: A rota rhs possui mais de um cliente e o cliente é o último da rota.
            else if (rhs_route.size() > 1 && rhs_client == rhs_route.size() - 1) {
              new_total_cost -= context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], kDepot);

              new_total_cost += context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], kDepot);
            }

            // Caso 2.4: A rota rhs possui mais de um cliente e o cliente está no meio da rota.
            else if (rhs_route.size() > 1 && rhs_client > 0 && rhs_client < rhs_route.size() - 1) {
              new_total_cost -= context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            } else {
              throw std::runtime_error("[inter-route] Caso 2.5: não tratado.");
            }
          }

          // Caso 3: A rota lhs possui mais de um cliente e o cliente é o último da rota.
          else if (lhs_route.size() > 1 && lhs_client == lhs_route.size() - 1) {
            new_total_cost -= context.distance(lhs_route[lhs_client - 1], lhs_route[lhs_client]);

            new_total_cost += context.distance(lhs_route[lhs_client - 1], rhs_route[rhs_client]);

            // Caso 3.1: A rota rhs possui apenas um cliente.
            if (rhs_route.size() == 1) {
              new_total_cost -= context.distance(kDepot, rhs_route[rhs_client]);

              new_total_cost += context.distance(kDepot, lhs_route[lhs_client]);
            }

            // Caso 3.2: A rota rhs possui mais de um cliente e o cliente é o primeiro da rota.
            else if (rhs_route.size() > 1 && rhs_client == 0) {
              new_total_cost -= context.distance(lhs_route[lhs_client], kDepot);
              new_total_cost -= context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(rhs_route[rhs_client], kDepot);
              new_total_cost += context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            }

            // Caso 3.3: A rota rhs possui mais de um cliente e o cliente é o último da rota.
            else if (rhs_route.size() > 1 && rhs_client == rhs_route.size() - 1) {
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);

              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
            }

            // Caso 3.4: A rota rhs possui mais de um cliente e o cliente está no meio da rota.
            else if (rhs_route.size() > 1 && rhs_client > 0 && rhs_client < rhs_route.size() - 1) {
              new_total_cost -= context.distance(lhs_route[lhs_client], kDepot);
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(rhs_route[rhs_client], kDepot);
              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            } else {
              throw std::runtime_error("[inter-route] Caso 3.5: não tratado.");
            }
          }

          // Caso 4: A rota lhs possui mais de um cliente e o cliente está no meio da rota.
          else if (lhs_route.size() > 1 && lhs_client > 0 && lhs_client < lhs_route.size() - 1) {
            new_total_cost -= context.distance(lhs_route[lhs_client - 1], lhs_route[lhs_client]);
            new_total_cost -= context.distance(lhs_route[lhs_client], lhs_route[lhs_client + 1]);

            new_total_cost += context.distance(lhs_route[lhs_client - 1], rhs_route[rhs_client]);
            new_total_cost += context.distance(rhs_route[rhs_client], lhs_route[lhs_client + 1]);

            // Caso 4.1: A rota rhs possui apenas um cliente.
            if (rhs_route.size() == 1) {
              new_total_cost -= context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], kDepot);

              new_total_cost += context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], kDepot);
            }

            // Caso 4.2: A rota rhs possui mais de um cliente e o cliente é o primeiro da rota.
            else if (rhs_route.size() > 1 && rhs_client == 0) {
              new_total_cost -= context.distance(kDepot, rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(kDepot, lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            }

            // Caso 4.3: A rota rhs possui mais de um cliente e o cliente é o último da rota.
            else if (rhs_route.size() > 1 && rhs_client == rhs_route.size() - 1) {
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], kDepot);

              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], kDepot);
            }

            // Caso 4.4: A rota rhs possui mais de um cliente e o cliente está no meio da rota.
            else if (rhs_route.size() > 1 && rhs_client > 0 && rhs_client < rhs_route.size() - 1) {
              new_total_cost -= context.distance(rhs_route[rhs_client - 1], rhs_route[rhs_client]);
              new_total_cost -= context.distance(rhs_route[rhs_client], rhs_route[rhs_client + 1]);

              new_total_cost += context.distance(rhs_route[rhs_client - 1], lhs_route[lhs_client]);
              new_total_cost += context.distance(lhs_route[lhs_client], rhs_route[rhs_client + 1]);
            } else {
              throw std::runtime_error("[inter-route] Caso 4.5: não tratado.");
            }
          } else {
            throw std::runtime_error("[inter-route] Caso 5: não tratado.");
          }

          // É melhor que o candidato atual?
          if (new_total_cost < std::get<2>(best_move)) {
            best_move = std::make_tuple(&lhs_route[lhs_client], &rhs_route[rhs_client], new_total_cost);
          }
        }
      }
    }
  }

  auto& [best_lhs_client, best_rhs_client, best_cost] = best_move;

  // Verifica se a solução atual é melhor que o candidato.
  if (best_cost >= best_solution.total_cost) {
    return best_solution;
  }

  if (s_debug) {
    std::cout << "vnd: swapping clients inter-route " << *best_lhs_client << " and " << *best_rhs_client << " ("
              << best_cost - best_solution.total_cost << ")" << std::endl;
  }

  // Troca os clientes e atualiza a solução atual.
  std::swap(*best_lhs_client, *best_rhs_client);
  best_solution = apa::rebuild_stats(context, best_solution);

  if (best_solution.total_cost != best_cost) {
    std::cout << "best_cost: " << best_cost << " best_solution.total_cost: " << best_solution.total_cost << std::endl;
    throw std::runtime_error("[inter-route] best_cost != best_solution.total_cost");
  }

  return best_solution;
}

apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};

  // Verifica se a solução atual já atingiu o limite de terceirização permitido (N - L).
  if (best_solution.outsourced.size() == static_cast<std::size_t>(context.outsourcing_threshold())) {
    return best_solution;
  }

  // Melhor candidato à terceirização.
  std::tuple<client, vehicle, cost> best_move{0, 0, std::numeric_limits<cost>::max()};

  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    std::vector<client>& route{best_solution.routes[vehicle]};

    for (std::size_t client_index = 0; client_index < route.size(); client_index++) {
      int new_total_cost{best_solution.total_cost};

      // Caso 1: A rota possui um único cliente.
      if (route.size() == 1) {
        new_total_cost -= context.distance(kDepot, route[client_index]);
        new_total_cost -= context.distance(route[client_index], kDepot);

        // O veículo não possui mais clientes.
        new_total_cost -= context.vehicle_cost;
      }
      // Caso geral: A rota possui mais de um cliente.
      else {
        // Caso geral 1: O cliente é o primeiro da rota.
        if (client_index == 0) {
          new_total_cost -= context.distance(kDepot, route[client_index]);
          new_total_cost -= context.distance(route[client_index], route[client_index + 1]);

          new_total_cost += context.distance(kDepot, route[client_index + 1]);
        }

        // Caso geral 2: O cliente é o último da rota.
        if (client_index == route.size() - 1) {
          new_total_cost -= context.distance(route[client_index - 1], route[client_index]);
          new_total_cost -= context.distance(route[client_index], kDepot);

          new_total_cost += context.distance(route[client_index - 1], kDepot);
        }

        // Caso geral 3: O cliente está no meio da rota.
        if (client_index > 0 && client_index < route.size() - 1) {
          new_total_cost -= context.distance(route[client_index - 1], route[client_index]);
          new_total_cost -= context.distance(route[client_index], route[client_index + 1]);

          new_total_cost += context.distance(route[client_index - 1], route[client_index + 1]);
        }
      }

      // Adiciona o custo de terceirização do cliente.
      new_total_cost += context.outsourcing_cost(route[client_index]);

      // É melhor que o candidato atual?
      if (new_total_cost < std::get<2>(best_move)) {
        best_move = std::make_tuple(route[client_index], vehicle, new_total_cost);
      }
    }
  }

  const auto& [best_client, best_vehicle, best_cost] = best_move;

  // Verifica se a solução atual é melhor que o candidato.
  if (best_cost >= best_solution.total_cost) {
    return best_solution;
  }

  if (s_debug) {
    std::cout << "vnd: outsourcing client " << best_client << " (" << best_cost - best_solution.total_cost << ")"
              << std::endl;
  }

  // Remove o cliente da rota do veículo.
  best_solution.routes[best_vehicle].erase(
      std::find(best_solution.routes[best_vehicle].begin(), best_solution.routes[best_vehicle].end(), best_client));

  // Adiciona o cliente à lista de clientes terceirizados e atualiza a solução atual.
  best_solution.outsourced.push_back(best_client);
  best_solution = apa::rebuild_stats(context, best_solution);

  if (best_solution.total_cost != best_cost) {
    throw std::runtime_error("[outsourcing] best_cost != best_solution.total_cost");
  }

  return best_solution;
}

int sum_vehicle_load(const apa::context& context, const std::vector<client>& route) {
  int load{};

  for (const client client : route) {
    load += context.demand(client);
  }

  return load;
}

vehicle find_best_vehicle(const std::vector<capacity>& capacities) {
  std::size_t vehicle_index{};
  for (std::size_t index = 0; index < capacities.size(); index++) {
    if (capacities[index] > capacities[vehicle_index]) {
      vehicle_index = index;
    }
  }

  return static_cast<int>(vehicle_index);
}

client find_last_client(const std::vector<std::vector<client>>& routes, vehicle vehicle) {
  return routes[vehicle].empty() ? kDepot  // A rota está vazia, então o cliente é o depósito (kDepot).
                                 : routes[vehicle].back();
}