#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <utility>

namespace apa {

class stopwatch {
 public:
  explicit stopwatch(std::string label)
      : _label{std::move(label)}, _start{std::chrono::high_resolution_clock::now()} {}

  ~stopwatch() {
    const auto end{std::chrono::high_resolution_clock::now()};
    const auto duration{std::chrono::duration_cast<std::chrono::milliseconds>(end - _start)};

    std::cout << _label << ":\t" << duration.count() << "ms" << std::endl;
  }

 private:
  std::string _label;
  std::chrono::time_point<std::chrono::high_resolution_clock> _start;
};

}  // namespace apa