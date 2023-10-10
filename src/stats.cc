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

int stats::count_used_vehicles() const {
  int used_vehicle{0};
  for (const auto& route : routes) {
    if (!route.empty()) {
      used_vehicle++;
    }
  }

  return used_vehicle;
}

int stats::recalculate_total_cost(const apa::context& context) const {
  int total_routing_cost{};
  int total_vehicle_cost{};
  int total_outsourcing_cost{};

  // Calcula o custo de roteamento.
  for (const auto& route : routes) {
    if (route.empty()) {
      continue;
    }

    int prev_client{0};  // O depósito (0) é o ponto de partida.
    for (int client : route) {
      total_routing_cost += context.distance(prev_client, client);
      prev_client = client;
    }

    // Adicionar o custo de retorno ao depósito
    total_routing_cost += context.distance(prev_client, 0);
  }

  // Calcula o custo de terceirização.
  for (const auto& client : outsourced_clients) {
    total_outsourcing_cost += context.outsourcing_cost(client);
  }

  // Calcula o custo de utilização dos veículos.
  for (const auto& route : routes) {
    if (!route.empty()) {
      total_vehicle_cost += context.vehicle_cost;
    }
  }

  return total_routing_cost + total_vehicle_cost + total_outsourcing_cost;
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