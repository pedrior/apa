#include "include/context.hh"

#include <fstream>
#include <stdexcept>
#include <string>

namespace apa {

std::ostream& operator<<(std::ostream& os, const context& context) {
  os << context.clients << "\n"
     << context.vehicles << "\n"
     << context.vehicle_capacity << "\n"
     << context.delivery_threshold << "\n"
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

context context_parser::parse(const std::filesystem::path& file) {
  std::ifstream fs{file, std::ios::in};
  if (!fs.is_open()) {
    throw std::runtime_error{"Could not open file: " + file.string()};
  }

  int n, k, q, l, r;
  fs >> n >> k >> q >> l >> r;

  std::vector<int> d(n);
  for (int i = 0; i < n; ++i) {
    fs >> d[i];
  }

  std::vector<int> p(n);
  for (int i = 0; i < n; ++i) {
    fs >> p[i];
  }

  std::vector<std::vector<int>> c(n + 1, std::vector<int>(n + 1));
  for (int i = 0; i < n + 1; ++i) {
    for (int j = 0; j < n + 1; ++j) {
      fs >> c[i][j];
    }
  }

  return {
      n,  // clients
      k,  // vehicles
      q,  // vehicle_capacity
      l,  // non_outsource_clients
      r,  // vehicle_cost
      d,  // demands
      p,  // outsourcing_costs
      c   // distances
  };
}

}  // namespace apa