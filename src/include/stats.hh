#pragma once

#include <ostream>
#include <vector>

#include "context.hh"

namespace apa {

struct stats {
  int routing_cost;
  int vehicles_cost;
  int outsourcing_cost;
  std::vector<int> outsourced;
  std::vector<std::vector<int>> routes;

  friend std::ostream& operator<<(std::ostream& os, const stats& stats);

  [[nodiscard]] int total_cost() const { return routing_cost + vehicles_cost + outsourcing_cost; }
};

class stats_serializer {
 public:
  static void serialize(const stats& stats, const std::string& filename);
};

[[maybe_unused]] static const apa::stats rebuild_stats(const apa::context& context, const apa::stats& stats) {
  int total_routing_cost{};
  int total_vehicle_cost{};
  int total_outsourcing_cost{};

  for (const auto& route : stats.routes) {
    for (std::size_t client_index{}; client_index < route.size() - 1; client_index++) {
      total_routing_cost += context.distance(route[client_index], route[client_index + 1]);
    }

    if (route.size() > 2) {
      total_vehicle_cost += context.vehicle_cost;
    }
  }

  // Calcula o custo de terceirização.
  for (const int client : stats.outsourced) {
    total_outsourcing_cost += context.outsourcing_cost(client);
  }

  return {
      total_routing_cost,      // routing_cost
      total_vehicle_cost,      // vehicles_cost
      total_outsourcing_cost,  // outsourcing_cost
      stats.outsourced,        // outsourced
      stats.routes             // routes
  };
}

}  // namespace apa
