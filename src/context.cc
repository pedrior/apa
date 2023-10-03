#include "includes/context.hh"

#include <fstream>
#include <stdexcept>
#include <string>

namespace apa {

std::ostream& operator<<(std::ostream& os, const context& context) {
  os << context.clients + 1 << "\n"
     << context.vehicles << "\n"
     << context.vehicle_capacity << "\n"
     << context.non_outsource_clients << "\n"
     << context.vehicle_cost;

  os << "\n\n";
  for (auto demand : context.demands) {
    os << demand << " ";
  }

  os << "\n\n";
  for (auto cost : context.outsourcing_costs) {
    os << cost << " ";
  }

  os << "\n\n";
  for (const auto& arc : context.distances) {
    for (const auto cost : arc) {
      os << cost << " \t";
    }

    os << std::endl;
  }

  return os;
}

context context_parser::parse(const std::string& filename) {
  std::ifstream file{filename, std::ios::in};
  if (!file.is_open()) {
    throw std::runtime_error{"Could not open file: " + filename};
  }

  int n, k, q, l, r;
  file >> n >> k >> q >> l >> r;

  std::vector<int> d(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    file >> d[i];
  }

  std::vector<int> p(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    file >> p[i];
  }

  std::vector<std::vector<int>> c(n, std::vector<int>(n));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      file >> c[i][j];
    }
  }

  return {
      n - 1,  // clients
      k,      // vehicles
      q,      // vehicle_capacity
      l,      // non_outsource_clients
      r,      // vehicle_cost
      d,      // demands
      p,      // outsourcing_costs
      c       // distances
  };
}

}  // namespace apa