#include <iostream>
#include <fstream>
#include <vector>

#include "roundabout.hpp"
#include "roundabout_manager.hpp"
#include "vehicle.hpp"

int main() {
  roundabout r(8, { 10, 10, 10, 10, 10, 10, 10, 10 });

  // TODO: test wrap around
  std::vector<vehicle> vs;
  vs.push_back(vehicle({ 0, 0, 7, 0, 0 }));
  vs.push_back(vehicle({ 1, 4, 7, 3, 0 }));
  // vs.push_back(vehicle({ 2, 4, 7, 4, 0 }));
  // vs.push_back(vehicle({ 3, 6, 7, 6, 0 }));

  // std::cout << "Received Vehicles: \n";
  // for (auto& v: vs) {
  //   std::cout << v << std::endl;
  // }
  // std::cout << std::endl;

  roundabout_manager rm(r, vs);
  rm.solve();

  std::ofstream file;
  file.open("trajectories.txt");
  for (auto& v: vs) {
    // std::cout << "[Vehicle " << v.id << "]\n";
    for (auto& [_, t]: v.trajs) {
      file << t;
    }
    file << std::endl;
  }

  return 0;
}