#pragma once

#include <iostream>
#include <vector>

#include "trajectory.hpp"

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
std::ostream& operator<<(std::ostream& os, const vehicle& v);