#pragma once

#include <algorithm>
#include <iostream>
#include <optional>
#include <utility>

#include "roundabout.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  roundabout_manager(roundabout&, std::vector<vehicle>&);

  void solve();
  trajectory schedule_first_vehicle(section&);

private:
  roundabout _r;
  std::vector<vehicle>& _vs;
};
