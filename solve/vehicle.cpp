#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "constants.hpp"
#include "vehicle.hpp"

vehicle::vehicle(int i, int in, int out, double at, double cp, double iv): 
  id(i), entry(in), progress(in), exit(out), 
  arrival_time(at), current_position(cp), init_velocity(iv) {}

trajectory& vehicle::get_traj(int sec_id) {
  auto it = std::find_if(trajs.begin(), trajs.end(), [sec_id](auto& t) {
    return t.first == sec_id;
  });
  if (it == trajs.end()) {
    throw std::range_error("invalid section id");
  }
  return it->second;
}

// returns the trajectory of the vehicle if it travels
// at the max acceleration until the max velocity
trajectory vehicle::max_velocity(double length) const {
  trajectory t(arrival_time, current_position, current_position + length, init_velocity);

  // already at the max speed, don't accelerate
  if (init_velocity >= MAX_V) {
    double time_at_cst_v = length / MAX_V;
    t.push_sub_traj(arrival_time + time_at_cst_v, 0);
    return t;
  }

  // run at max acceleration until the max velocity
  double time_at_max_a = (MAX_V - init_velocity) / MAX_A;
  double dist_at_max_a = (MAX_V + init_velocity) * time_at_max_a / 2;

  // if the length is not enough to reach the max velocity
  if (dist_at_max_a >= length) {
    double time_travel = (sqrt(init_velocity * init_velocity + 2 * MAX_A * length) - init_velocity) / MAX_A;
    t.push_sub_traj(arrival_time + time_travel, MAX_A);
  }

  // reached max velocity, finish the rest with no acceleration
  else {
    double time_at_cst_v = (length - dist_at_max_a) / MAX_V;
    t.push_sub_traj(arrival_time + time_at_max_a, MAX_A);
    t.push_sub_traj(arrival_time + time_at_max_a + time_at_cst_v, 0);
  }

  return t;
}


std::ostream& operator<<(std::ostream& os, const subtrajectory& st) {
  os << "x=" << st.entry_position << "+" << st.entry_velocity << "\\left(y-" << st.entry_time << "\\right)+"
     << "\\frac{" << st.acc << "}{2}\\left(y-" << st.entry_time << "\\right)^{2}"
     << "\\left\\{" << st.entry_time << "\\le y<" << st.leave_time << "\\right\\}\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const trajectory& t) {
  for (auto& i: t.sub_trajs) {
    os << i;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const vehicle& v) {
  os << "[Vehicle " << v.id << "]\n"
     << "\tentry:          " << v.entry << "\n"
     << "\texit:           " << v.exit << "\n"
     << "\tarrival_time:   " << v.arrival_time << "\n"
     << "\tinit_velocity:  " << v.init_velocity << "\n";
  for (auto& [_, t]: v.trajs) {
    os << t;
  }

  return os;
}