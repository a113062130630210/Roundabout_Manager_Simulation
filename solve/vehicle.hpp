#pragma once

#include <iostream>
#include <vector>

#include "trajectory.hpp"
#include "types.hpp"

struct vehicle {
  vehicle(int, modular<int>, modular<int>, double, double, double);

  trajectory& get_traj(int);
  trajectory max_velocity(double) const;

  int id;
  int index;

  modular<int> entry;
  modular<int> progress;
  modular<int> exit;

  double arrival_time;
  double current_position;
  double init_velocity;
  std::vector<std::pair<int, trajectory>> trajs;
};
std::ostream& operator<<(std::ostream& os, const vehicle& v);