#pragma once

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../context.hh"
#include "../stats.hh"

namespace apa {

class algorithm {
 public:
  explicit algorithm(std::string name, apa::context context)
      : _name{std::move(name)}, _context{std::move(context)} {
    for (int client = _context.clients; client >= 1; client--) {
      _pending_clients.insert(client);
    }

    _vehicle_routes = std::vector<std::vector<int>>(_context.vehicles, std::vector<int>());
    _vehicle_capacities = std::vector<int>(_context.vehicles, _context.vehicle_capacity);
  }

  virtual ~algorithm() = default;

  [[nodiscard]] virtual apa::stats run() = 0;

  [[nodiscard]] const std::string& name() const { return _name; }

 protected:
  [[nodiscard]] apa::context& context() { return _context; }

  [[nodiscard]] int total_routing_cost() const { return _total_routing_cost; };

  [[nodiscard]] int total_outsourcing_cost() const { return _total_outsourcing_cost; };

  [[nodiscard]] const std::unordered_set<int>& pending_clients() const { return _pending_clients; };

  [[nodiscard]] const std::vector<int>& outsourced_clients() const { return _outsourced_clients; };

  [[nodiscard]] const std::vector<int>& vehicle_capacities() const { return _vehicle_capacities; };

  [[nodiscard]] const std::vector<std::vector<int>>& vehicle_routes() const {
    return _vehicle_routes;
  };

  [[nodiscard]] int total_vehicle_cost() const;

  [[nodiscard]] int total_cost() const;

  [[nodiscard]] bool has_pending_clients() const;

  [[nodiscard]] int find_vehicle_with_most_capacity() const;

  [[nodiscard]] int find_last_client_in_route(int vehicle) const;

  [[nodiscard]] int find_pending_client_with_lowest_outsource_cost();

  [[nodiscard]] int vehicle_capacity(int vehicle) const;

  void load_vehicle(int vehicle, int demand);

  void add_to_route(int vehicle, int client, int distance);

  void serve_client(int client);

  void outsource_client(int client);

  void return_to_depot(int vehicle);

 private:
  std::string _name;
  apa::context _context;

  int _total_routing_cost{0};
  int _total_outsourcing_cost{0};

  std::unordered_set<int> _pending_clients{};
  std::vector<int> _outsourced_clients{};
  std::vector<int> _vehicle_capacities;
  std::vector<std::vector<int>> _vehicle_routes;
};

}  // namespace apa
