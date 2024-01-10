#pragma once

#include <iostream>
#include <vector>

#include "types.hpp"

struct subtrajectory {
  subtrajectory
  (double, double, modular<double>, modular<double>, double, double, double);

  double entry_time;
  double leave_time;
  modular<double> entry_position;
  modular<double> leave_position;
  double entry_velocity;
  double leave_velocity;
  double acc;

  bool conflict_with(const subtrajectory&) const;
};

struct trajectory {
  trajectory(double, modular<double>, modular<double>, double);

  bool place_on_top(const trajectory&);
  bool avoid_front(const trajectory&);
  trajectory& push_sub_traj(double, double);

  double entry_time;
  double leave_time;
  modular<double> entry_position;
  modular<double> leave_position;
  double entry_velocity;
  double leave_velocity;
  std::vector<subtrajectory> sub_trajs;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);