#pragma once

#include <ostream>
#include <vector>

#include "context.hh"

namespace apa {

struct stats {
  int total_cost;
  int routing_cost;
  int vehicles_cost;
  int outsourcing_cost;
  std::vector<int> outsourced_clients;
  std::vector<std::vector<int>> routes;

  [[nodiscard]] int count_used_vehicles() const;

  friend std::ostream& operator<<(std::ostream& os, const stats& stats);
};

class stats_serializer {
 public:
  static void serialize(const stats& stats, const std::string& filename);
};

[[maybe_unused]] static const apa::stats update_stats(const apa::context& context, const apa::stats& stats) {
  int total_routing_cost{};
  int total_vehicle_cost{};
  int total_outsourcing_cost{};

  // Calcula o custo de roteamento.
  for (const auto& route : stats.routes) {
    if (route.empty()) {
      continue;
    }

    int prev_client{0};  // O depósito (0) é o ponto de partida.
    for (int client : route) {
      total_routing_cost += context.distance(prev_client, client);
      prev_client = client;
    }

    // Adiciona o custo de retorno ao depósito
    total_routing_cost += context.distance(prev_client, 0);
  }

  // Calcula o custo de terceirização.
  for (const auto& client : stats.outsourced_clients) {
    total_outsourcing_cost += context.outsourcing_cost(client);
  }

  // Calcula o custo de utilização dos veículos.
  for (const auto& route : stats.routes) {
    if (!route.empty()) {
      total_vehicle_cost += context.vehicle_cost;
    }
  }

  return {
      total_routing_cost + total_vehicle_cost + total_outsourcing_cost,  // total_cost
      total_routing_cost,                                                // routing_cost
      total_vehicle_cost,                                                // vehicles_cost
      total_outsourcing_cost,                                            // outsourcing_cost
      stats.outsourced_clients,                                          // outsourced_clients
      stats.routes                                                       // routes
  };
}

}  // namespace apa
