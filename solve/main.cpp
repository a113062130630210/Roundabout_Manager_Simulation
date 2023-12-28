#include <vector>

#include "constants.hpp"
#include "roundabout_manager.hpp"

int main(int argc, char *argv[]) {
  if (argc != 3) {
    EXIT("Please provide the output files.");
  }

  roundabout_manager rm;
  rm.load_input();
  rm.solve();
  rm.print_result(argv[1], argv[2]);

  return 0;
}