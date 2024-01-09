#include <vector>

#include "constants.hpp"
#include "roundabout_manager.hpp"

int main(int argc, char *argv[]) {
  if (argc != 4) {
    EXIT("Please provide required files.");
  }

  roundabout_manager rm;
  rm.load_input(argv[1]);
  rm.solve();
  rm.print_result(argv[2], argv[3]);

  return 0;
}