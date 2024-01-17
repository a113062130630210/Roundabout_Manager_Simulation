#include <cmath>
#include <iomanip>
#include <numeric>
#include <tuple>

#include "constants.hpp"
#include "trajectory.hpp"
#include "util.hpp"
#include "vehicle.hpp"

subtrajectory::subtrajectory
(double et, double lt, double ep, double lp, double ev, double lv, double a):
  entry_time(et), leave_time(lt), entry_position(ep), leave_position(lp), 
  entry_velocity(ev), leave_velocity(lv), acc(a) {}

bool subtrajectory::conflict_with(const subtrajectory& traj) const {
  // check if `traj` is above `*this` (only entry time is checked)
  {
    bool this_is_front = entry_position >= traj.entry_position;
    const subtrajectory& front = this_is_front ? *this : traj;
    const subtrajectory& back  = this_is_front ? traj : *this;

    if (back.acc == 0 && back.entry_velocity == 0) {
      EXIT("[ERROR] trajectory::is_conflict\nthis should not happen");
    }

    double d = front.entry_position - back.entry_position;
    double a = back.acc;
    double v = back.entry_velocity;
    double t = back.entry_time;

    double delta = v*v + 2*a*d;
    if (fabs(delta) <= 1e-10) delta = 0;
    double time_at_front_position = (a == 0 ? d/v : (sqrt(delta) - v) / a) + t;

    if (
      (this_is_front && time_at_front_position + TIME_GAP > entry_time) ||
      (!this_is_front && traj.entry_time + TIME_GAP > time_at_front_position)
    ) return true;
  }

  // check if `*this` and `traj` intersects
  auto s = quadratic_solver(
         entry_time           ,      entry_position,      entry_velocity,      acc, 
    traj.entry_time + TIME_GAP, traj.entry_position, traj.entry_velocity, traj.acc
  );
  if (!s) return false;

  // check if any of the intersection points is in the range of trajectories
  double l_bound = std::max(entry_time, traj.entry_time + TIME_GAP);
  double r_bound = std::min(leave_time, traj.leave_time + TIME_GAP);
  return (
    (l_bound < s->first && s->first < r_bound) || 
    (l_bound < s->second && s->second < r_bound)
  );
};


trajectory::trajectory(double t, double ep, double lp, double v): 
  version(0), entry_time(t), leave_time(t), entry_position(ep), 
  leave_position(lp), entry_velocity(v), 
  leave_velocity(v), sub_trajs(), front_traj_ver(0), front_traj(nullptr) {}

// the trajectory should directly come from vehicle::max_velocity
bool trajectory::place_on_top(const trajectory& t) {
  if (sub_trajs.empty() || t.sub_trajs.empty()) {
    EXIT("[ERROR] place_on_top\ngot empty list");
  }

  for (auto s_it = sub_trajs.begin(); s_it != sub_trajs.end(); s_it++) {
    for (auto t_it = t.sub_trajs.begin(); t_it != t.sub_trajs.end(); t_it++) {
      if (t_it->leave_position + 1e-10 < s_it->entry_position) continue;

      auto solutions = tangent_solver(
        s_it->entry_time, s_it->entry_position, 
        s_it->entry_velocity, s_it->acc, 
        t_it->entry_time + TIME_GAP, t_it->entry_position, 
        t_it->entry_velocity, t_it->acc
      );
      if (solutions) {
        auto& [sol1, sol2] = solutions.value();
        std::optional<std::pair<double, double>> result;
        if (
          s_it->entry_time <= sol1.first && 
          sol1.first <= sol1.second && 
          sol1.first <= s_it->leave_time &&
          sol1.second <= t_it->leave_time + TIME_GAP
        ) {
          result = sol1;
        }
        else if (
          s_it->entry_time <= sol2.first && 
          sol2.first <= sol2.second && 
          sol2.first <= s_it->leave_time &&
          sol2.second <= t_it->leave_time + TIME_GAP
        ) {
          result = sol2;
        }

        if (result) {
          auto [t_star, t_tang] = result.value();
          double tmp_acc = s_it->acc;

          wipe_trajs(s_it);
          push_sub_traj(t_star, tmp_acc);
          push_sub_traj(t_tang, MIN_A);
          push_sub_traj(t_it->leave_time + TIME_GAP, t_it->acc);

          for (auto it = t_it + 1; it != t.sub_trajs.end(); it++) {
            push_sub_traj(it->leave_time + TIME_GAP, it->acc);
          }

          version++;
          front_traj = &t;
          front_traj_ver = t.version;
          return true;
        }
      }
      
      bool b = s_it->conflict_with(*t_it);
      if (b) return false;
    }
  }
  
  return true;
}

std::optional<trajectory>
trajectory::avoid_front(trajectory target, double ring_len) {
  // temparary instance to help the calculation
  vehicle veh(-1, {1, -1}, {1, -1}, leave_time, leave_position, leave_velocity);
  double length = target.leave_position - leave_position;
  if (length < 0) {
    length += ring_len;
    target.entry_position += ring_len;
    target.leave_position += ring_len;
    for (auto& st: target.sub_trajs) {
      st.entry_position += ring_len;
      st.leave_position += ring_len;
    }
  }

  auto nxt_traj = veh.max_velocity(length);
  if (nxt_traj.place_on_top(target)) {
    return nxt_traj;
  }

  // the distance between the end of the section &
  // the start posiion of current sub-trajectory
  double distance_left = 0;

  // start from the last sub-trajectory
  for (auto it = sub_trajs.rbegin(); it != sub_trajs.rend(); ++it) {
    distance_left += it->leave_position - it->entry_position;

    // binary search the best timing to decelerate
    bool too_slow = false;
    std::optional<std::pair<double, double>> best_sol;
    std::optional<trajectory> following;

    double b = it->entry_time;
    double e = it->leave_time;
    double m = b;

    for (int i = 0; i < 25 && b != e; i++) {
      double t = m - it->entry_time;                                      // total time to travel before deceleration
      double v = it->entry_velocity + it->acc*t;                          // initial velocity of the deceleration process
      double d = distance_left - (it->entry_velocity*t + it->acc*t*t/2);  // distance left for the deceleration process

      // check if initial velocity `v` is too slow to travel the distance `d`
      double delta = v*v + 2*MIN_A*d;
      if (delta < 1e-10) {
        too_slow = true;
        b = m;
      }

      // `v` is fast enough to make it
      else {
        if (fabs(delta) <= 1e-10) delta = 0;
        t  = (-v + sqrt(delta)) / MIN_A;  // the travel time of current sub-trajectory
        v += MIN_A * t;                   // the leave velocity of current sub-trajectory

        veh.arrival_time  = m + t;
        veh.init_velocity = v;
        nxt_traj = veh.max_velocity(length);

        // the next trajactory can be legally put on top of the target, 
        // set the current state as best solution, decelerate later
        if (nxt_traj.place_on_top(target)) {
          best_sol = { m, m + t };
          following = nxt_traj;
          b = m;
        }

        // otherwise, decelerate earlier
        else {
          e = m;
        }
      }

      m = (b + e) / 2;
    }

    if (best_sol) {
      double acc = sub_trajs.back().acc;
      sub_trajs.pop_back();
      push_sub_traj(best_sol->first, acc);
      push_sub_traj(best_sol->second, MIN_A);
      return following;
    }
    if (too_slow) return std::nullopt;

    sub_trajs.pop_back();
  }

  return std::nullopt;
}

void trajectory::split(split_iter iter, const split_iter& end) const {
  auto st_iter = sub_trajs.cbegin();
  iter->second.entry_time = entry_time;
  iter->second.entry_velocity = entry_velocity;
  iter->second.clear_trajs();
  iter->second.push_sub_traj(-1, st_iter->acc);

  while (st_iter != sub_trajs.cend()) {
    if (iter->second.leave_time > st_iter->leave_time) {
      iter->second.sub_trajs.pop_back();
      iter->second.push_sub_traj(st_iter->leave_time, st_iter->acc);
      ++st_iter;
      iter->second.push_sub_traj(-1, st_iter->acc);
    }
    else {
      iter->second.version++;
      ++iter;
      if (iter == end) return;
      iter->second.entry_time = (iter-1)->second.leave_time;
      iter->second.entry_velocity = (iter-1)->second.leave_velocity;
      iter->second.clear_trajs();
      iter->second.push_sub_traj(-1, st_iter->acc);
    }
  }

  throw std::logic_error("this should not happen");
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
    double delta = ev*ev + 2 * acc * (leave_position - ex);
    if (delta <= 1e-10) delta = 0; // if delta < 0, decelerate until v = 0
    end_time = et + (sqrt(delta) - ev) / acc;
  }

  if (fabs(end_time - et) <= 1e-10) return *this;
  if (end_time < et) {
    EXIT("[ERROR] trajectory::push_sub_traj\ninfeasible end time");
  }

  double t = end_time - et;
  double lx = ex + ev * t + acc * t * t / 2;
  double lv = ev + acc * t;

  if (lx > leave_position + 1e-10) {
    EXIT("[ERROR] trajectory::push_sub_traj\nposition out of range");
  }

  leave_time = end_time;
  leave_velocity = lv;
  sub_trajs.emplace_back(et, end_time, ex, lx, ev, lv, acc);

  return *this;
}

void trajectory::clear_trajs() {
  leave_time = entry_time;
  sub_trajs.clear();
}

void trajectory::wipe_trajs(const sub_trajs_t::iterator& start) {
  leave_time = start->entry_time;
  for (auto it = sub_trajs.end() - 1; it >= start; it--) {
    sub_trajs.pop_back();
  }
}
