#include "vehicle.hpp"

subtrajectory::subtrajectory(double et, double lt, double ep, double lp, double ev, double lv, double a):
  entry_time(et), leave_time(lt), entry_position(ep), leave_position(lp), entry_velocity(ev), leave_velocity(lv), acc(a) {}

bool subtrajectory::conflict_with(const subtrajectory& t) const {
  if (entry_position <= t.entry_position) {
    double distance = t.entry_position - entry_position;
    double time;
    if (acc == 0) {
      if (entry_velocity == 0) EXIT("[ERROR] trajectory::is_conflict ??? 1");
      time = distance / entry_velocity;
    }
    else {
      time = (-entry_velocity + sqrt(entry_velocity*entry_velocity + 2*acc*distance)) / acc;
    }
    if (time + entry_time < t.entry_time + TIME_GAP) return true;
  }
  else {
    double distance = entry_position - t.entry_position;
    double time;
    if (t.acc == 0) {
      if (t.entry_velocity == 0) EXIT("[ERROR] trajectory::is_conflict ??? 2");
      time = distance / t.entry_velocity;
    }
    else {
      time = (-t.entry_velocity + sqrt(t.entry_velocity*t.entry_velocity + 2*t.acc*distance)) / t.acc;
    }
    if (entry_time < time + t.entry_time + TIME_GAP) return true;
  }

  if (leave_position <= t.leave_position) {
    double distance = leave_position - t.entry_position;
    double time;
    if (t.acc == 0) {
      if (t.entry_velocity == 0) EXIT("[ERROR] trajectory::is_conflict ??? 3");
      time = distance / t.entry_velocity;
    }
    else {
      time = (-t.entry_velocity + sqrt(t.entry_velocity*t.entry_velocity + 2*t.acc*distance)) / t.acc;
    }
    if (leave_time < time + t.entry_time + TIME_GAP) return true;
  }
  else {
    double distance = t.leave_position - entry_position;
    double time;
    if (acc == 0) {
      if (entry_velocity == 0) EXIT("[ERROR] trajectory::is_conflict ??? 4");
      time = distance / entry_velocity;
    }
    else {
      time = (-entry_velocity + sqrt(entry_velocity*entry_velocity + 2*acc*distance)) / acc;
    }
    if (time + entry_time < t.leave_time + TIME_GAP) return true;
  }

  auto s = quadratic_solver(
    entry_time             ,   entry_position,   entry_velocity,   acc, 
    t.entry_time + TIME_GAP, t.entry_position, t.entry_velocity, t.acc
  );
  if (!s) return false;

  double l_bound = std::max(entry_time, t.entry_time);
  double r_bound = std::max(leave_time, t.leave_time);
  return (l_bound < s->first && s->first < r_bound) || (l_bound < s->second && s->second < r_bound);
};


trajectory::trajectory(bool e, double t, double ep, double lp, double v)
  : is_entry(e), entry_time(t), leave_time(t)
  , entry_position(ep), leave_position(lp), entry_velocity(v), leave_velocity(v)
  , sub_trajs(std::vector<subtrajectory>()) {}

// the trajectory should directly come from vehicle::max_velocity
bool trajectory::place_on_top(const trajectory& t) {
  if (sub_trajs.empty() || t.sub_trajs.empty()) {
    EXIT("[ERROR] place_on_top\nlist empty");
  }
  if (entry_position != t.entry_position) {
    EXIT("[ERROR] place_on_top\nposition not aligned");
  }

  auto s_it = sub_trajs.begin();
  auto t_it = t.sub_trajs.begin();

  // check if any two sub-trajectories conflict
  while (s_it != sub_trajs.end() || t_it != t.sub_trajs.end()) {
    // calculate the intersection points of two (quadratic) trajectories
    if (s_it->conflict_with(*t_it)) {
      auto solutions = tangent_solver(
        s_it->entry_time           , s_it->entry_position, s_it->entry_velocity, s_it->acc, 
        t_it->entry_time + TIME_GAP, t_it->entry_position, t_it->entry_velocity, t_it->acc
      );

      if (!solutions) return false;

      auto& [sol1, sol2] = solutions.value();
      std::optional<std::pair<double, double>> pair = std::nullopt;
      if (s_it->entry_time <= sol1.first && sol1.first <= sol1.second) {
        pair = sol1;
      }
      else if (s_it->entry_time <= sol2.first && sol2.first <= sol2.second) {
        pair = sol2;
      }

      if (!pair) return false;

      sub_trajs.erase(s_it, sub_trajs.end());

      auto [t_star, t_int] = pair.value();
      double tmp_acc = s_it->acc;
      push_sub_traj(t_star, tmp_acc);

      if (t_int <= t_it->leave_time + TIME_GAP) {
        push_sub_traj(t_int, MIN_A);
        push_sub_traj(t_it->leave_time + TIME_GAP, t_it->acc);
      }
      else {
        push_sub_traj(-1, MIN_A);
      }


      for (auto it = t_it + 1; it != t.sub_trajs.end(); it++) {
        push_sub_traj(it->leave_time + TIME_GAP, it->acc);
      }
      return true;
    }

    double slp = s_it->leave_position;
    double tlp = t_it->leave_position;
    if (slp <= tlp + 1e-10) s_it++;
    if (slp + 1e-10 >= tlp) t_it++;

    if (s_it > sub_trajs.end() || t_it > t.sub_trajs.end()) {
      EXIT("[ERROR] place_on_top exceed boundary");
    }
  }

  return true;
}

bool trajectory::avoid_front(const trajectory& target) {
  // temparary instances to help the calculation
  const double length = target.leave_position - target.entry_position;
  vehicle veh(-1, -1, -1, leave_time, leave_velocity);
  veh.current_position = leave_position;

  auto nxt_traj = veh.max_velocity(length);
  if (nxt_traj.place_on_top(target)) return true;

  double distance_left = 0; // the distance between the end of the section & the start posiion of current sub-trajectory

  // start from the last sub-trajectory
  auto it = sub_trajs.end() - 1;
  while (true) {
    if (it < sub_trajs.begin()) break;

    distance_left += it->leave_position - it->entry_position;

    // binary search the best timing to decelerate
    bool too_slow = false;
    auto best_sol = std::make_pair(-1., -1.);

    {
      double b = it->entry_time;
      double e = it->leave_time;
      double m = b;
      for (int i = 0; i < 25 && b != e; i++) {
        double t = m - it->entry_time;                                      // total time to travel before deceleration
        double v = it->entry_velocity + it->acc*t;                          // initial velocity of the deceleration process
        double d = distance_left - (it->entry_velocity*t + it->acc*t*t/2);  // distance left for the deceleration process

        // check if initial velocity `v` is too slow to travel the distance `d`
        double delta = v*v + 2*MIN_A*d;
        if (delta < 0) {
          too_slow = true;
          b = m;
        }

        // `v` is fast enough to make it
        else {
          t  = (-v + sqrt(delta)) / MIN_A;  // the travel time of current sub-trajectory
          v += MIN_A * t;                   // the leave velocity of current sub-trajectory

          veh.arrival_time  = m + t;
          veh.init_velocity = v;
          nxt_traj = veh.max_velocity(length);

          // the next trajactory can be legally put on top of the target, 
          // set the current state as best solution, decelerate later
          if (nxt_traj.place_on_top(target)) {
            best_sol = std::make_pair(m, m + t);
            b = m;
          }

          // otherwise, decelerate earlier
          else {
            e = m;
          }
        }

        m = (b + e) / 2;
      }
    }

    // best solution is found
    if (best_sol.first > 0) {
      double acc = sub_trajs.back().acc;
      sub_trajs.pop_back();
      push_sub_traj(best_sol.first, acc);
      push_sub_traj(best_sol.second, MIN_A);
      return true;
    }

    // the vehicle is not able to make it
    if (too_slow) return false;

    sub_trajs.pop_back();    
    it = sub_trajs.end() - 1;
  }

  return false;
}

trajectory& trajectory::push_sub_traj(double end_time, double acc) {
  double et, ex, ev;
  if (sub_trajs.empty()) {
    et = entry_time;
    ex = entry_position;
    ev = entry_velocity;
  }
  else {
    subtrajectory& st = sub_trajs.back();
    et = st.leave_time;
    ex = st.leave_position;
    ev = st.leave_velocity;
  }

  if (end_time < 0) {
    end_time = et + (sqrt(ev*ev + 2 * acc * (leave_position - ex)) - ev) / acc;
  }

  if (fabs(end_time - et) <= 1e-10) return *this;
  if (end_time < et) {
    EXIT("[RANGE_ERROR] trajectory::push_sub_traj 2");
  }

  double t = end_time - et;
  double lx = ex + ev * t + acc * t * t / 2;
  double lv = ev + acc * t;

  if (lx > leave_position + 1e-10) {
    EXIT("[RANGE_ERROR] trajectory::push_sub_traj\nposition out of range");
  }

  leave_time = end_time;
  leave_velocity = lv;
  sub_trajs.emplace_back(et, end_time, ex, lx, ev, lv, acc);

  return *this;
}


vehicle::vehicle(int i, int in, int out, double at, double iv)
  : id(i), entry(in), exit(out), arrival_time(at), init_velocity(iv) {}

// returns the trajectory of the vehicle if it travels
// at the max acceleration until the max velocity
trajectory vehicle::max_velocity(double length) const {
  trajectory t(false, arrival_time, current_position, current_position + length, init_velocity);

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