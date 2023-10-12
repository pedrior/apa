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
 * @param iteration_threshold Número máximo de iterações sem melhoria.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Realiza movimentos envolvendo clientes na mesma rota.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_intra_route(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Realiza movimentos envolvendo clientes entre rotas.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_inter_routes(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo terceirização de clientes.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Calcula a carga de um veículo.
 * @param context Instância do problema.
 * @param route Rota do veículo.
 * @return Carga total do veículo dado sua rota.
 */
int vehicle_load(const apa::context& context, const std::vector<client>& route);

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
  const auto& vnd_stats{variable_neighborhood_descent(context, greedy_stats)};

  const std::string& filename{input_file.stem().string()};

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

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution{initial_solution};

  // Estruturas de vizinhança.
  std::vector<apa::stats (*)(const apa::context&, const apa::stats&)> neighborhoods = {
      move_client_intra_route, move_client_inter_routes, move_client_with_outsourcing};

  std::size_t neighborhood_index{};
  while (neighborhood_index < neighborhoods.size()) {
    const auto& neighborhood{neighborhoods[neighborhood_index]};

    // Aplica a estrutura de vizinhança na melhor solução encontrada até o momento.
    apa::stats current_solution{neighborhood(context, best_solution)};

    // Avalia a solução atual.
    if (current_solution.total_cost < best_solution.total_cost) {
      best_solution = current_solution;

      // Reseta o índice da estrutura de vizinhança para que todas as estruturas de vizinhança sejam aplicadas na nova
      // melhor solução.
      neighborhood_index = 0;
    } else {
      if (s_debug) {
        std::cout << "vnd: no improvement in "
                  << (neighborhood_index == 0   ? "intra-routes"
                      : neighborhood_index == 1 ? "inter-routes"
                                                : "outsourcing")
                  << " neighborhood" << std::endl;
      }

      // Incrementa o índice da estrutura de vizinhança. Se o índice igualar ao tamanho do vetor de estruturas de
      // vizinhança, não há mais melhorias possíveis (todas as estruturas de vizinhança foram aplicadas). Neste caso,
      // o algoritmo termina com a melhor solução encontrada.
      neighborhood_index++;
    }
  }

  if (s_debug) {
    std::cout << "vnd: total cost gain: " << best_solution.total_cost - initial_solution.total_cost << std::endl;
  }

  return best_solution;
}

apa::stats move_client_intra_route(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution = initial_solution;

  // Itera sobre as rotas.
  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    apa::stats current_solution = best_solution;
    std::vector<client>& route{current_solution.routes[vehicle]};

    // Itera sobre os pares de clientes possíveis na rota.
    for (std::size_t lhs_client_index = 0; lhs_client_index < route.size(); lhs_client_index++) {
      for (std::size_t rhs_client_index = lhs_client_index + 1; rhs_client_index < route.size(); rhs_client_index++) {
        // Troca os clientes nos índices lhs_client_index e rhs_client_index.
        std::swap(route[lhs_client_index], route[rhs_client_index]);

        // Atualiza os custos e avalia a solução atual.
        current_solution = apa::rebuild_stats(context, current_solution);
        if (current_solution.total_cost < best_solution.total_cost) {
          if (s_debug) {
            std::cout << "vnd: swapping clients intra-route " << route[rhs_client_index] << " and "
                      << route[lhs_client_index] << " (" << current_solution.total_cost - best_solution.total_cost
                      << ")" << std::endl;
          }

          best_solution = current_solution;
        } else {
          // A troca não melhorou a solução, então desfaz a troca para que a próxima iteração possa examinar outro par
          // de clientes possíveis.
          std::swap(route[lhs_client_index], route[rhs_client_index]);
        }
      }
    }
  }

  return best_solution;
}

apa::stats move_client_inter_routes(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution = initial_solution;

  // Itera sobre todos os pares de veículos possíveis.
  for (vehicle lhs_vehicle = 0; lhs_vehicle < context.vehicles; lhs_vehicle++) {
    for (vehicle rhs_vehicle = lhs_vehicle + 1; rhs_vehicle < context.vehicles; rhs_vehicle++) {
      apa::stats current_solution = best_solution;

      // Obtém as rotas dos veículos.
      std::vector<client>& lhs_route = current_solution.routes[lhs_vehicle];
      std::vector<client>& rhs_route = current_solution.routes[rhs_vehicle];

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

          // Troca os clientes lhs_client e rhs_client.
          std::swap(lhs_client, rhs_client);

          // Atualiza os custos e avalia a solução atual.
          current_solution = apa::rebuild_stats(context, current_solution);
          if (current_solution.total_cost < best_solution.total_cost) {
            if (s_debug) {
              std::cout << "vnd: swapping clients inter-route " << rhs_client << " and " << lhs_client << " ("
                        << current_solution.total_cost - best_solution.total_cost << ")" << std::endl;
            }

            best_solution = current_solution;
          } else {
            // A troca não melhorou a solução, então desfaz a troca para que a próxima iteração possa examinar outro par
            // de clientes possíveis.
            std::swap(lhs_client, rhs_client);
          }
        }
      }
    }
  }

  return best_solution;
}

apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution = initial_solution;

  std::vector<std::pair<client, int>> candidates{};

  // Itera sobre as rotas.
  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    apa::stats current_solution = best_solution;
    std::vector<client>& route{current_solution.routes[vehicle]};
    std::vector<client>& outsourced_clients{current_solution.outsourced_clients};

    // Itera sobre os clientes da rota.
    for (const client client : route) {
      // Verifica se a solução atual atingiu o limite de terceirizações.
      if (static_cast<int>(outsourced_clients.size()) == context.outsourcing_threshold()) {
        return current_solution;
      }

      // Encontra o índice do cliente na rota atual.
      const auto& client_index = std::find(route.begin(), route.end(), client) - route.begin();

      // Remove o cliente da rota atual.
      route.erase(route.begin() + client_index);

      // Insere o cliente removido na lista de clientes terceirizados.
      outsourced_clients.insert(outsourced_clients.begin(), client);

      // Atualiza os custos e avalia a solução atual.
      current_solution = apa::rebuild_stats(context, current_solution);
      if (current_solution.total_cost < best_solution.total_cost) {
        if (s_debug) {
          std::cout << "vnd: outsource client " << client << " ("
                    << current_solution.total_cost - best_solution.total_cost << ")" << std::endl;
        }

        best_solution = current_solution;
      } else {
        // A terceirização não melhorou a solução, então desfaz a terceirização para que a próxima iteração possa
        // examinar outro cliente.
        outsourced_clients.erase(outsourced_clients.begin());
        route.insert(route.begin() + client_index, client);
      }
    }
  }

  return best_solution;
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