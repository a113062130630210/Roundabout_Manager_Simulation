#include <iostream>
#include <vector>

#include "roundabout.hpp"
#include "roundabout_manager.hpp"
#include "vehicle.hpp"

int main() {
  roundabout r(8, { 10, 10, 10, 10, 10, 10, 10, 10 });

  // TODO: test wrap around
  std::vector<vehicle> vs;
  vs.push_back(vehicle({ 0, 0, 5, 0, 0 }));
  // vs.push_back(vehicle({ 1, 2, 7, 2, 0 }));
  // vs.push_back(vehicle({ 2, 4, 7, 4, 0 }));

  for (auto& v: vs) {
    std::cout << v << std::endl;
  }

  roundabout_manager rm(r, vs);
  rm.solve();

  std::cout << std::endl;
  for (auto& v: vs) {
    std::cout << "[Vehicle " << v.id << "]\n";
    for (auto& [_, t]: v.trajs) {
      std::cout << t;
    }
    std::cout << std::endl;
  }

  return 0;
}