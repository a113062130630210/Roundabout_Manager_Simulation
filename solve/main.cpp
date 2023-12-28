#include <vector>

#include "constants.hpp"
#include "roundabout.hpp"
#include "roundabout_manager.hpp"
#include "vehicle.hpp"

int main(int argc, char *argv[]) {
  if (argc != 3) {
    EXIT("Please provide the output files.");
  }

  roundabout r(8, { 10, 10, 10, 10, 10, 10, 10, 10 });

  // TODO: test wrap around
  std::vector<vehicle> vs;
  vs.push_back(vehicle({ 0, 6, 7, 6.4, 0 }));
  vs.push_back(vehicle({ 1, 0, 7, 0, 0 }));

  roundabout_manager rm(r, vs);
  rm.solve();
  rm.print_result(argv[1], argv[2]);

  return 0;
}