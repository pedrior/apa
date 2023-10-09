#pragma once

#include <ostream>
#include <string>
#include <vector>

namespace apa {

struct context {
  int clients;                              // n
  int vehicles;                             // k
  int vehicle_capacity;                     // Q
  int non_outsource_clients;                // L
  int vehicle_cost;                         // r
  std::vector<int> demands;                 // d
  std::vector<int> outsourcing_costs;       // p
  std::vector<std::vector<int>> distances;  // c

  [[nodiscard]] int distance(int origin, int target) const { return distances[origin][target]; }

  [[nodiscard]] int demand(int client) const { return demands[client - 1]; }

  [[nodiscard]] int outsourcing_cost(int client) const { return outsourcing_costs[client - 1]; }

  friend std::ostream& operator<<(std::ostream& os, const context& context);
};

class context_parser {
 public:
  [[nodiscard]] static context parse(const std::string& filename);
};

}  // namespace apa
