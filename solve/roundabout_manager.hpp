#pragma once

#include "roundabout.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  roundabout_manager(roundabout&, std::vector<vehicle>&);

  void solve();
  void print_result();

private:
  roundabout _r;
  std::vector<vehicle> _vs;
  bool _solved;

  trajectory schedule_first_vehicle(section&);
};
