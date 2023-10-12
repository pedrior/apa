#include "includes/stats.hh"

#include <fstream>

namespace apa {

std::ostream& operator<<(std::ostream& os, const stats& stats) {
  os << stats.total_cost << "\n"
     << stats.routing_cost << "\n"
     << stats.vehicles_cost << "\n"
     << stats.outsourcing_cost << "\n";

  os << "\n";

  for (const auto& client : stats.outsourced_clients) {
    os << client << " ";
  }

  int used_routes{0};
  for (const auto& route : stats.routes) {
    if (!route.empty()) {
      used_routes++;
    }
  }

  os << "\n\n" << used_routes << "\n";

  for (const auto& route : stats.routes) {
    for (const auto& client : route) {
      os << client << " ";
    }

    os << "\n";
  }

  return os;
}

void stats_serializer::serialize(const stats& stats, const std::string& filename) {
  std::ofstream file{filename, std::ios::out | std::ios::trunc};
  if (!file.is_open()) {
    throw std::runtime_error{"Could not open file '" + filename + "' for writing"};
  }

  file << stats;
  file.close();

  if (!file.good()) {
    throw std::runtime_error{"Could not write to file '" + filename + "'"};
  }
}

}  // namespace apa