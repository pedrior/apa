#include "../includes/algorithms/greedy.hh"

#include <iostream>
#include <limits>

namespace apa {

apa::stats greedy::run() {
  while (greedy::has_pending_clients()) {
    const int vehicle{greedy::find_vehicle_with_most_capacity()};
    const int origin_client{greedy::find_last_client_in_route(vehicle)};
    const int target_client{greedy::find_highest_priority_client(origin_client, vehicle)};

    if (target_client) {
      const int target_demand{context().demand(target_client)};
      const int target_distance{context().distance(origin_client, target_client)};

      greedy::load_vehicle(vehicle, target_demand);
      greedy::add_to_route(vehicle, target_client, target_distance);
      greedy::serve_client(target_client);
    } else {
      greedy::outsource_client(greedy::find_pending_client_with_lowest_outsource_cost());
    }
  }

  // All clients have been served, return to depot.
  for (int vehicle = 0; vehicle < context().vehicles; vehicle++) {
    greedy::return_to_depot(vehicle);
  }

  return {
      greedy::total_cost(),              // total_cost
      greedy::total_routing_cost(),      // routing_cost
      greedy::total_vehicle_cost(),      // vehicles_cost
      greedy::total_outsourcing_cost(),  // outsourcing_cost
      greedy::outsourced_clients(),      // outsourced_clients
      greedy::vehicle_routes()           // vehicle_routes
  };
}

constexpr float kDistanceWeight{0.1};
constexpr float kDemandWeight{0.5};
constexpr float kOutsourcingWeight{0.4};

int greedy::find_highest_priority_client(int origin, int vehicle) {
  int client{0};
  float min_cost{std::numeric_limits<float>::max()};

  for (const auto &target : pending_clients()) {
    const int demand{context().demand(target)};
    const int distance{context().distance(origin, target)};
    const int outsourcing_cost{context().outsourcing_cost(target)};

    // cost = α * distance + β * demand + γ * outsourcing_cost
    float cost{static_cast<float>((float)distance * kDistanceWeight /
                                  (float)demand * kDemandWeight +
                                  (float)outsourcing_cost * kOutsourcingWeight)};

    if (cost < min_cost && demand <= greedy::vehicle_capacity(vehicle)) {
      client = target;
      min_cost = cost;
    }
  }

  return client;
}

}  // namespace apa