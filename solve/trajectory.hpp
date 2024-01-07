#pragma once

#include <iostream>
#include <vector>

struct subtrajectory {
  subtrajectory(double, double, double, double, double, double, double);

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
  trajectory(double, double, double, double);

  bool place_on_top(const trajectory&);
  bool avoid_front(const trajectory&);
  trajectory& push_sub_traj(double, double);

  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  std::vector<subtrajectory> sub_trajs;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);