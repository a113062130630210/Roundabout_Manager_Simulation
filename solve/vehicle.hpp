#pragma once

#include <iostream>
#include <map>

#include "trajectory.hpp"

struct vehicle {
  vehicle(int, int, int, double, double);

  trajectory max_velocity(double) const;
  void insert_trajectory(int, const trajectory&);

  int id;
  int index;
  int entry;
  int exit;
  double arrival_time;
  double current_position;
  double init_velocity;
  std::map<int, trajectory> trajs;
};
std::ostream& operator<<(std::ostream& os, const vehicle& v);