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

  for (const auto& route : stats.routes) {
    if (route.empty()) {
      continue;
    }

    int origin_client{0};
    for (const int target_client : route) {
      total_routing_cost += context.distance(origin_client, target_client);  // Adiciona o custo de roteamento.
      origin_client = target_client;
    }

    total_vehicle_cost += context.vehicle_cost;                // Adiciona o custo de utilização do veículo.
    total_routing_cost += context.distance(origin_client, 0);  // Adiciona o custo de retorno ao depósito.
  }

  // Calcula o custo de terceirização.
  for (const int client : stats.outsourced_clients) {
    total_outsourcing_cost += context.outsourcing_cost(client);
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
