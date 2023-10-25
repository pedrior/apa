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

}  // namespace apa
