#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include "roundabout.hpp"
#include "roundabout_manager.hpp"
#include "vehicle.hpp"

int main() {
  roundabout r(8, { 10, 10, 10, 10, 10, 10, 10, 10 });

  // generate possion arrival test case
  std::vector<double> arrival_times;
  std::default_random_engine generator;
  std::exponential_distribution<double> distribution(MEAN_INTERARRIVAL_TIME);
  for (int number_of_vehicles = 1 ; number_of_vehicles <= 5 ; number_of_vehicles++) {
      double arrival_time_diff = distribution(generator);
      double arrival_time = 0;
      if (arrival_times.size() == 0) {
          arrival_times.push_back(arrival_time);
      }
      else {
          arrival_times.push_back(arrival_time[arrival_times.size() - 1] + arrival_time_diff);
      }
  }

  // TODO: test wrap around
  std::vector<vehicle> vs;
  vs.push_back(vehicle({ 0, 6, 7, 6.4, 0 }));
  vs.push_back(vehicle({ 1, 0, 7, 0, 0 }));

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