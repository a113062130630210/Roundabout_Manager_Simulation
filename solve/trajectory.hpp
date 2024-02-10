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
public:
  using t_iter = std::vector<trajectory>::iterator;
  using st_iter = std::vector<subtrajectory>::iterator;

  trajectory(double, double, double, double);

  std::optional<trajectory> place_on_top(trajectory, double) const;
  bool conflict_with(trajectory) const;
  bool conflict_with(subtrajectory) const;
  trajectory& push_sub_traj(double, double, bool = true);
  trajectory& push_sub_traj(double, double, double, bool = true);
  void split(t_iter, const t_iter&) const;
  void clear_trajs();

  int version;
  double entry_time;
  double leave_time;
  double entry_position;
  double leave_position;
  double entry_velocity;
  double leave_velocity;
  std::vector<subtrajectory> sub_trajs;

  int front_traj_ver;
  trajectory const * front_traj;

private:
  std::optional<std::pair<double, double>>
  find_tangent(const st_iter&, const st_iter&, const st_iter&) const;
  std::optional<double>
  find_point(const st_iter&, const st_iter&) const;
};

std::ostream& operator<<(std::ostream& os, const subtrajectory& st);
std::ostream& operator<<(std::ostream& os, const trajectory& t);