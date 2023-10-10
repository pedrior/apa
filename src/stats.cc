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

int stats::count_used_routes() const {
  int used_routes{0};
  for (const auto& route : routes) {
    if (!route.empty()) {
      used_routes++;
    }
  }

  return used_routes;
}

int stats::recalculate_total_cost(const apa::context& context) const {
  int total_cost_internal{};

  // Calcula o custo de roteamento.
  for (const auto& route : routes) {
    for (std::size_t i = 0; i < route.size() - 1; i++) {
      // Adiciona o custo de ir de um cliente ao próximo na rota atual.
      total_cost_internal += context.distance(route[i], route[i + 1]);
    }

    // Se a rota não estiver vazia, adiciona o custo de ir do último cliente ao primeiro (de volta ao depósito).
    if (!route.empty()) {
      total_cost_internal += context.distance(route.back(), route.front());
    }
  }

  // Calcula o custo de terceirização.
  for (const auto& client : outsourced_clients) {
    total_cost_internal += context.outsourcing_cost(client);
  }

  // Calcula o custo de utilização dos veículos.
  for (const auto& route : routes) {
    if (route.empty()) {
      continue;
    }

    total_cost_internal += context.vehicle_cost;
  }

  return total_cost_internal;
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