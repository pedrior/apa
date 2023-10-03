#include "../includes/algorithms/algorithm.hh"

namespace apa {

int algorithm::total_vehicle_cost() const {
  int total_vehicle_cost{};
  for (const auto &capacity : _vehicle_capacities) {
    if (capacity < _context.vehicle_capacity) {
      total_vehicle_cost += _context.vehicle_cost;
    }
  }

  return total_vehicle_cost;
}

int algorithm::total_cost() const {
  return algorithm::total_routing_cost() + algorithm::total_outsourcing_cost() +
         algorithm::total_vehicle_cost();
}

bool algorithm::has_pending_clients() const { return !_pending_clients.empty(); }

int algorithm::find_vehicle_with_most_capacity() const {
  int vehicle{0};
  for (int current_vehicle = 0; current_vehicle < _context.vehicles; current_vehicle++) {
    if (_vehicle_capacities[current_vehicle] > _vehicle_capacities[vehicle]) {
      vehicle = current_vehicle;
    }
  }

  return vehicle;
}

void algorithm::load_vehicle(int vehicle, int demand) { _vehicle_capacities[vehicle] -= demand; }

void algorithm::add_to_route(int vehicle, int client, int distance) {
  _vehicle_routes[vehicle].push_back(client);
  _total_routing_cost += distance;
}

void algorithm::serve_client(int client) { _pending_clients.erase(client); }

int algorithm::find_last_client_in_route(int vehicle) const {
  return _vehicle_routes[vehicle].empty() ? 0 : _vehicle_routes[vehicle].back();
}

int algorithm::find_pending_client_with_lowest_outsource_cost() {
  int client{*_pending_clients.begin()};

  for (const auto &current : _pending_clients) {
    if (_context.outsourcing_cost(current) < _context.outsourcing_cost(client)) {
      client = current;
    }
  }

  return client;
}

int algorithm::vehicle_capacity(int vehicle) const { return _vehicle_capacities[vehicle]; }

void algorithm::outsource_client(int client) {
  _pending_clients.erase(client);
  _outsourced_clients.push_back(client);

  _total_outsourcing_cost += _context.outsourcing_cost(client);
}

void algorithm::return_to_depot(int vehicle) {
  if (!_vehicle_routes[vehicle].empty()) {
    _total_routing_cost += _context.distance(_vehicle_routes[vehicle].back(), 0);
  }
}

}  // namespace apa