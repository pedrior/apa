#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

namespace apa {

static std::unordered_map<std::string, int> s_optimal{
    {"n9k5_A", 428},     {"n9k5_B", 506},    {"n9k5_C", 559},    {"n9k5_D", 408},     {"n14k5_A", 471},
    {"n14k5_B", 565},    {"n14k5_C", 569},   {"n14k5_D", 471},   {"n22k3_A", 605},    {"n22k3_B", 777},
    {"n22k3_C", 777},    {"n22k3_D", 605},   {"n31k5_A", 650},   {"n31k5_B", 933},    {"n31k5_C", 939},
    {"n31k5_D", 656},    {"n43k6_A", 801},   {"n43k6_B", 1203},  {"n43k6_C", 1208},   {"n43k6_D", 802},
    {"n64k9_A", 934},    {"n64k9_B", 1503},  {"n64k9_C", 1510},  {"n64k9_D", 932},    {"n120k7_A", 1029},
    {"n120k7_B", 2052},  {"n120k7_C", 2040}, {"n120k7_D", 1046}, {"n199k17_A", 1672}, {"n199k17_B", 3302},
    {"n199k17_C", 3301}, {"n199k17_D", 1672}};

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