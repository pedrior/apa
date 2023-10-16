#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

namespace apa {

static std::unordered_map<std::string, int> s_optimal{
    {"n9k5_A", 428},  {"n9k5_B", 506},  {"n9k5_C", 559},  {"n9k5_D", 408},  {"n14k5_A", 471}, {"n14k5_B", 565},
    {"n14k5_C", 569}, {"n14k5_D", 471}, {"n22k3_A", 605}, {"n22k3_B", 777}, {"n22k3_C", 777}, {"n22k3_D", 605}};

static double gap(const std::string& target, double heuristic_value) {
  auto it = s_optimal.find(target);
  if (it == s_optimal.end()) {
    std::cout << "Error: target " << target << " not found in optimal values" << std::endl;
    return 0.0;
  }

  const int optimal_value{it->second};
  return ((heuristic_value - optimal_value) / optimal_value) * 100;
}

}  // namespace apa