#pragma once

#include <iostream>
#include <optional>
#include <vector>

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
  using split_iter = std::vector<std::pair<int, trajectory>>::iterator;

  trajectory(double, double, double, double);

  std::optional<trajectory> place_on_top(trajectory, double) const;
  trajectory& push_sub_traj(double, double);
  void split(split_iter, const split_iter&) const;
  void clear_trajs();

  int version;
  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  sub_trajs_t sub_trajs;

  int front_traj_ver;
  trajectory const * front_traj;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);