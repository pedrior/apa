#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "includes/context.hh"
#include "includes/stats.hh"

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

/**
 * @brief Algoritmo guloso para o problema de roteamento de veículos.
 * @param context Instância do problema.
 * @return Estatísticas da solução encontrada.
 */
apa::stats greedy(const apa::context& context);

/**
 * @brief Encontra o próximo cliente a ser atendido dado um cliente de origem e a capacidade do veículo.
 * @param context Instância do problema.
 * @param pending_clients Conjunto de clientes pendentes.
 * @param origin Cliente de origem.
 * @param vehicle_capacity Capacidade do veículo para o qual o cliente será atendido.
 * @return Próximo cliente a ser atendido. Se não houver cliente possível, retorna -1 (não há cliente possível).
 */
client greedy_next_client(const apa::context& context, const pending_clients& pending_clients, client origin,
                          capacity vehicle_capacity);

/**
 * @brief Melhora a solução inicial utilizando estruturas de vizinhança.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats exhaustive_neighborhood_search(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo clientes na mesma rota.
 * @param context Instância do problema.
 * @param solution Estatísticas da solução atual.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_within_route(const apa::context& context, const apa::stats& solution);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo clientes em rotas diferentes.
 * @param context Instância do problema.
 * @param solution Estatísticas da solução atual.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_between_routes(const apa::context& context, const apa::stats& solution);

/**
 * @brief Busca o último cliente atendido na rota de um veículo.
 * @param routes Lista de rotas dos veículos.
 * @param vehicle Veículo a ser considerado.
 * @return Último cliente atendido na rota do veículo. Se a rota estiver vazia, retorna 0 (depósito).
 */
client find_last_client_in_route(const routes& routes, vehicle vehicle);

/**
 * @brief Encontra o veículo com maior capacidade disponível.
 * @param vehicle_capacities Lista de capacidades dos veículos.
 * @return Veículo com maior capacidade disponível.
 */
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
  const auto& context = apa::context_parser::parse(input_file);

  // Acho que o parser está OK. Qualquer coisa, descomenta aqui e verifica se a instância foi lida corretamente.
  //  if (s_debug) {
  //    std ::cout << context << std::endl;
  //  }

  const auto& greedy_stats{greedy(context)};
  const auto& exhaustive_stats{exhaustive_neighborhood_search(context, greedy_stats)};

  const std::string& filename{input_file.stem().string()};

  // Estamos salvando as estatísticas em dois arquivos diferentes para facilitar a comparação entre as soluções.
  // Mas no final, só precisamos da solução golusa que será combinada com as melhorias da busca exaustiva.
  apa::stats_serializer::serialize(greedy_stats, filename + std::string("_greedy.txt"));
  apa::stats_serializer::serialize(exhaustive_stats, filename + std::string("_greedy2.txt"));

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

apa::stats exhaustive_neighborhood_search(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution{initial_solution};

  // Vizinhança 1: move clientes numa única rota.
  best_solution = move_client_within_route(context, best_solution);
  const int nb_1_cost_gain{best_solution.total_cost - initial_solution.total_cost};

  if (s_debug) {
    std::cout << "ex_nbs_sr: best solution difference: " << nb_1_cost_gain << std::endl;
  }

  // Vizinhança 2: move clientes em múltiplas rotas.
  best_solution = move_client_between_routes(context, best_solution);
  const int nb_2_cost_gain{best_solution.total_cost - initial_solution.total_cost - nb_1_cost_gain};

  if (s_debug) {
    std::cout << "ex_nbs_mr: best solution difference: " << nb_2_cost_gain << std::endl;
  }

  return best_solution;
}

apa::stats move_client_within_route(const apa::context& context, const apa::stats& solution) {
  // Melhor solução inicial é a solução atual
  apa::stats best_solution{solution};

  const std::size_t used_vehicle_count{static_cast<std::size_t>(solution.count_used_vehicles())};
  for (std::size_t vehicle = 0; vehicle < used_vehicle_count; vehicle++) {
    const auto& route{best_solution.routes[vehicle]};
    const std::size_t route_clients_count{route.size()};

    // Examina todos os pares de clientes possíveis
    for (std::size_t lhs_client = 0; lhs_client < route_clients_count; lhs_client++) {
      for (std::size_t rhs_client = lhs_client + 1; rhs_client < route_clients_count; rhs_client++) {
        apa::stats new_solution{best_solution};

        // Troca os clientes nas posições lhs_client e rhs_client na mesma rota.
        std::swap(new_solution.routes[vehicle][lhs_client], new_solution.routes[vehicle][rhs_client]);

        // Atualiza os custos da nova solução.
        new_solution.total_cost = new_solution.recalculate_total_cost(context);
        new_solution.routing_cost =
            new_solution.total_cost - new_solution.outsourcing_cost - new_solution.vehicles_cost;
        new_solution.outsourcing_cost =
            new_solution.total_cost - new_solution.routing_cost - new_solution.vehicles_cost;

        // Se a nova solução for melhor que a melhor solução atual, atualiza a melhor solução.
        if (new_solution.total_cost < best_solution.total_cost) {
          if (s_debug) {
            std::cout << "ex_nbs_sr: new solution found by swapping clients "
                      << new_solution.routes[vehicle][rhs_client] << " and " << new_solution.routes[vehicle][lhs_client]
                      << " in route with vehicle " << vehicle
                      << ".\tCost gain: " << new_solution.total_cost - best_solution.total_cost << std::endl;
          }

          best_solution = new_solution;
        }
      }
    }
  }

  return best_solution;
}

apa::stats move_client_between_routes(const apa::context& context, const apa::stats& solution) {
  apa::stats best_solution{solution};  // Melhor solução inicial é a solução atual

  const std::size_t routes_count{best_solution.routes.size()};

  // Examina todos os pares de rotas possíveis
  for (std::size_t lhs_vehicle = 0; lhs_vehicle < routes_count; ++lhs_vehicle) {
    for (std::size_t rhs_vehicle = lhs_vehicle + 1; rhs_vehicle < routes_count; ++rhs_vehicle) {
      // 'move_client_within_route' já faz a troca de clientes dentro da mesma rota.
      if (lhs_vehicle == rhs_vehicle) {
        continue;
      }

      apa::stats new_solution{best_solution};

      std::vector<client>& lhs_route = new_solution.routes[lhs_vehicle];
      std::vector<client>& rhs_route = new_solution.routes[rhs_vehicle];

      // Examina todos os pares de clientes possíveis entre as rotas lhs_route e rhs_route.
      for (client& lhs_client : lhs_route) {
        for (client& rhs_client : rhs_route) {
          // Troca os clientes lhs_client e rhs_client nas suas respectivas rotas lhs_route e rhs_route.
          std::swap(lhs_client, rhs_client);

          // Atualiza os custos da nova solução.
          new_solution.total_cost = new_solution.recalculate_total_cost(context);
          new_solution.routing_cost =
              new_solution.total_cost - new_solution.outsourcing_cost - new_solution.vehicles_cost;
          new_solution.outsourcing_cost =
              new_solution.total_cost - new_solution.routing_cost - new_solution.vehicles_cost;

          if (new_solution.total_cost < best_solution.total_cost) {
            if (s_debug) {
              std::cout << "ex_nbs_mr: new solution found by swapping clients " << rhs_client << " and " << lhs_client
                        << " between routes with vehicles " << lhs_vehicle << " and " << rhs_vehicle
                        << ".\tCost gain: " << new_solution.total_cost - best_solution.total_cost << std::endl;
            }

            best_solution = new_solution;
          } else {
            // A troca não melhorou a solução, então desfaz a troca.
            std::swap(lhs_client, rhs_client);
          }
        }
      }
    }
  }

  return best_solution;
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