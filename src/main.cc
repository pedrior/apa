#include <iostream>
#include <memory>
#include <vector>

#include "includes/algorithms/greedy.hh"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <input.txt>" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<std::unique_ptr<apa::algorithm>> algorithms{};
  apa::context context = apa::context_parser::parse(argv[1]);

  std::cout << context << std::endl;

  algorithms.push_back(std::make_unique<apa::greedy>(context));

  for (const auto& algorithm : algorithms) {
    const auto& algorithm_stats{algorithm->run()};
    const auto& algorithm_name{algorithm->name()};

    apa::stats_serializer::serialize(algorithm_stats, algorithm_name + apa::kStatsFileExtension);
  }

  return EXIT_SUCCESS;
}