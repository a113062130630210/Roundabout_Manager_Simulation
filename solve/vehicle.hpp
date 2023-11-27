#pragma once

#include <cmath>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

#include "constants.hpp"
#include "util.hpp"

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
  trajectory(bool, double, double, double, double);

  bool place_on_top(const trajectory&);
  bool avoid_front(const trajectory&);
  trajectory& push_sub_traj(double, double);

  bool is_entry; // TODO: for future features
  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  std::vector<subtrajectory> sub_trajs;
};

struct vehicle {
  vehicle(int, int, int, double, double);

  trajectory max_velocity(double) const;

  int id;
  int entry;
  int exit;
  double arrival_time;
  double current_position;
  double init_velocity;
  std::vector<std::pair<int, trajectory>> trajs;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);
std::ostream& operator<<(std::ostream& os, const vehicle& v);