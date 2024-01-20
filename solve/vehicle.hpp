#pragma once

#include <iostream>
#include <vector>

#include "trajectory.hpp"
#include "modular.hpp"

struct vehicle {
  vehicle(int, int, modular<int>, modular<int>, double, double, double);

  trajectory& get_traj(int);
  trajectory max_velocity(double) const;

  const int id;
  const int index;

  const modular<int> entry;
  const modular<int> exit;
  modular<int> progress;

  double entry_time;
  double cur_pos;
  double entry_velocity;
  std::vector<std::pair<int, trajectory>> trajs;
};
std::ostream& operator<<(std::ostream& os, const vehicle& v);