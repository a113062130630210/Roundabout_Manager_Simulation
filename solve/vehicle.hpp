#pragma once

#include <iostream>
#include <vector>

#include "trajectory.hpp"
#include "modular.hpp"

struct vehicle {
  vehicle(int, int, modular<int>, modular<int>, double, double, double);

  std::vector<trajectory>::iterator get_traj(const modular<int>&);
  void max_velocity(trajectory&, double) const;

  const int id;
  const int index;

  const modular<int> entry;
  const modular<int> exit;
  modular<int> progress;

  double entry_time;
  double cur_pos;
  double entry_velocity;
  std::vector<trajectory> trajs;
};
std::ostream& operator<<(std::ostream& os, const vehicle& v);