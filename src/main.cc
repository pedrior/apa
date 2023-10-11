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

// Número máximo de iterações sem melhoria para o algoritmo VND.
constexpr std::size_t kVndIterationThreshold{50};

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
apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& initial_solution,
                                         std::size_t iteration_threshold);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo clientes na mesma rota.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_within_route(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo clientes em rotas diferentes.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_between_routes(const apa::context& context, const apa::stats& initial_solution);

/**
 * @brief Estrutura de vizinhança que realiza movimentos envolvendo terceirização de clientes.
 * @param context Instância do problema.
 * @param initial_solution Estatísticas da solução inicial.
 * @return Estatísticas da melhor solução encontrada.
 */
apa::stats move_client_with_outsourcing(const apa::context& context, const apa::stats& initial_solution);

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
  const auto& vnd_stats{variable_neighborhood_descent(context, greedy_stats, kVndIterationThreshold)};

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

apa::stats variable_neighborhood_descent(const apa::context& context, const apa::stats& initial_solution,
                                         std::size_t iteration_threshold) {
  apa::stats best_solution{initial_solution};  // A solução inicial é a melhor solução.
  std::size_t iteration{};                     // Número de iterações sem melhoria.

  // Estruturas de vizinhança.
  std::vector<apa::stats (*)(const apa::context&, const apa::stats&)> neighborhoods = {
      move_client_within_route, move_client_between_routes, move_client_with_outsourcing};

  // Enquanto o número de iterações sem melhoria for menor que o limiar...
  while (iteration < iteration_threshold) {
    for (const auto& neighborhood : neighborhoods) {
      // Aplica a estrutura de vizinhança na melhor solução encontrada até o momento.
      apa::stats current_solution{neighborhood(context, best_solution)};

      // Se a solução atual for melhor que a melhor solução encontrada até o momento, atualiza a melhor solução.
      if (current_solution.total_cost < best_solution.total_cost) {
        best_solution = current_solution;

        // Reseta o número de iterações sem melhoria.
        iteration = 0;
      } else {
        iteration++;
      }

      if (s_debug) {
        std::cout << "vnd: no improvement in neighborhood "
                  << (neighborhood == move_client_within_route     ? "sr"
                      : neighborhood == move_client_between_routes ? "mr"
                                                                   : "os")
                  << ".\tIteration: " << iteration << std::endl;
      }
    }
  }

  if (s_debug) {
    std::cout << "vnd: total cost difference:\t" << best_solution.total_cost - initial_solution.total_cost << std::endl;
  }

  return best_solution;
}

apa::stats move_client_within_route(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution{initial_solution};

  // Para cada veículo, isto é, para cada rota...
  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    const auto& route{best_solution.routes[vehicle]};

    // Se a rota estiver vazia, não há clientes para serem movidos.
    if (route.empty()) {
      continue;
    }

    // Examina todos os pares de clientes possíveis
    for (std::size_t lhs_client = 0; lhs_client < route.size(); lhs_client++) {
      for (std::size_t rhs_client = lhs_client + 1; rhs_client < route.size(); rhs_client++) {
        apa::stats current_solution{best_solution};

        // Troca os clientes nas posições lhs_client e rhs_client.
        std::swap(current_solution.routes[vehicle][lhs_client], current_solution.routes[vehicle][rhs_client]);

        // Atualiza os custos da nova solução.
        current_solution = apa::update_stats(context, current_solution);

        // Se a solução atual for melhor que a melhor solução encontrada até o momento, atualiza a melhor solução.
        if (current_solution.total_cost < best_solution.total_cost) {
          if (s_debug) {
            std::cout << "vnd-sr: swapping clients " << current_solution.routes[vehicle][rhs_client] << " and "
                      << current_solution.routes[vehicle][lhs_client] << " in route with vehicle " << vehicle
                      << ".\tCost gain: " << current_solution.total_cost - best_solution.total_cost << std::endl;
          }

          best_solution = current_solution;
        }
      }
    }
  }

  return best_solution;
}

apa::stats move_client_between_routes(const apa::context& context, const apa::stats& initial_solution) {
  apa::stats best_solution{initial_solution};

  // Para cada par de veículos, isto é, para cada par de rotas...
  for (vehicle lhs_vehicle = 0; lhs_vehicle < context.vehicles; lhs_vehicle++) {
    for (vehicle rhs_vehicle = lhs_vehicle + 1; rhs_vehicle < context.vehicles; rhs_vehicle++) {
      // Caso em que as rotas são iguais, já foram examinadas na vizinhança 1 e não precisam ser examinadas novamente?
      if (lhs_vehicle == rhs_vehicle) {
        continue;
      }

      apa::stats current_solution{best_solution};

      std::vector<client>& lhs_route = current_solution.routes[lhs_vehicle];
      std::vector<client>& rhs_route = current_solution.routes[rhs_vehicle];

      // Examina todos os pares de clientes possíveis entre as rotas lhs_route e rhs_route.
      for (client& lhs_client : lhs_route) {
        for (client& rhs_client : rhs_route) {
          // Troca os clientes lhs_client e rhs_client. Observe que estamos trocando os clientes por referência.
          std::swap(lhs_client, rhs_client);

          // Atualiza os custos da nova solução.
          current_solution = apa::update_stats(context, current_solution);

          // Se a solução atual for melhor que a melhor solução encontrada até o momento, atualiza a melhor solução.
          if (current_solution.total_cost < best_solution.total_cost) {
            if (s_debug) {
              std::cout << "vnd-mr: swapping clients " << rhs_client << " and " << lhs_client
                        << " between routes with vehicles " << lhs_vehicle << " and " << rhs_vehicle
                        << ".\tCost gain: " << current_solution.total_cost - best_solution.total_cost << std::endl;
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
  apa::stats best_solution{initial_solution};

  // Para cada veículo, isto é, para cada rota...
  for (vehicle vehicle = 0; vehicle < context.vehicles; vehicle++) {
    // Para cada cliente na rota...
    for (const client client : best_solution.routes[vehicle]) {
      apa::stats current_solution{best_solution};

      // Encontra o índice do cliente na rota atual.
      const auto& client_index =
          std::find(current_solution.routes[vehicle].begin(), current_solution.routes[vehicle].end(), client) -
          current_solution.routes[vehicle].begin();

      // Remove o cliente da rota atual.
      current_solution.routes[vehicle].erase(current_solution.routes[vehicle].begin() + client_index);

      // Insere o cliente removido na lista de clientes terceirizados.
      current_solution.outsourced_clients.insert(current_solution.outsourced_clients.begin(), client);

      // Atualiza os custos da nova solução.
      current_solution = apa::update_stats(context, current_solution);

      // Se a solução atual for melhor que a melhor solução encontrada até o momento, atualiza a melhor solução.
      if (current_solution.total_cost < best_solution.total_cost) {
        if (s_debug) {
          std::cout << "vnd-os: outsource client " << client << " from route with vehicle " << vehicle
                    << ".\tCost gain: " << current_solution.total_cost - best_solution.total_cost << std::endl;
        }

        best_solution = current_solution;
      } else {
        // A terceirização não melhorou a solução, então desfaz a terceirização para que a próxima iteração possa
        // examinar outro cliente.
        current_solution.outsourced_clients.erase(current_solution.outsourced_clients.begin());
        current_solution.routes[vehicle].insert(current_solution.routes[vehicle].begin() + client_index, client);
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