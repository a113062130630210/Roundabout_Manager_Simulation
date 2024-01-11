#pragma once

#include <iostream>
#include <vector>

#include "types.hpp"

struct subtrajectory {
  subtrajectory
  (double, double, double, double, double, double, double);

  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  double acc;

  bool conflict_with(const subtrajectory&) const;
};

struct trajectory {
  using sub_trajs_t = std::vector<subtrajectory>;

  trajectory(double, double, double, double);

  bool place_on_top(const trajectory&);
  bool avoid_front(trajectory, double);
  trajectory& push_sub_traj(double, double);
  void clear_trajs();
  void wipe_trajs(const sub_trajs_t::iterator&);

  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  sub_trajs_t sub_trajs;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);