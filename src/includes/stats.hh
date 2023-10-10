#pragma once

#include <ostream>
#include <vector>

#include "context.hh"

namespace apa {

static constexpr auto kStatsFileExtension = ".txt";

struct stats {
  int total_cost;
  int routing_cost;
  int vehicles_cost;
  int outsourcing_cost;
  std::vector<int> outsourced_clients;
  std::vector<std::vector<int>> routes;

  [[nodiscard]] int count_used_vehicles() const;

  [[nodiscard]] int recalculate_total_cost(const apa::context& context) const;

  friend std::ostream& operator<<(std::ostream& os, const stats& stats);
};

class stats_serializer {
 public:
  static void serialize(const stats& stats, const std::string& filename);
};

}  // namespace apa
