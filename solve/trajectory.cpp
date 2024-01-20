#include <algorithm>
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

bool subtrajectory::on_top_of(const subtrajectory& traj) const {
  // check if `traj` is above `*this` (only entry time is checked)
  {
    bool this_is_front = entry_position >= traj.entry_position;
    const subtrajectory& front = this_is_front ? *this : traj;
    const subtrajectory& back  = this_is_front ? traj : *this;

    if (back.acc == 0 && back.entry_velocity == 0) {
      EXIT("[ERROR] subtrajectory::on_top_of\nthis should not happen");
    }

    double d = front.entry_position - back.entry_position;
    double a = back.acc;
    double v = back.entry_velocity;
    double t = back.entry_time;

    double delta = v*v + 2*a*d;
    if (fabs(delta) <= 1e-10) delta = 0;
    double time_at_front_position = (a == 0 ? d/v : (sqrt(delta) - v) / a) + t;

    if (
      (this_is_front && time_at_front_position > entry_time) ||
      (!this_is_front && traj.entry_time > time_at_front_position)
    ) return true;
  }

  // check if `*this` and `traj` intersects
  auto s = quadratic_solver(
         entry_time,      entry_position,      entry_velocity,      acc, 
    traj.entry_time, traj.entry_position, traj.entry_velocity, traj.acc
  );
  if (!s) return false;

  // check if any of the intersection pnoints is in the range of trajectories
  double l_bound = std::max(entry_time, traj.entry_time);
  double r_bound = std::min(leave_time, traj.leave_time);
  return (
    (l_bound + 1e-10 < s->first && s->first + 1e-10 < r_bound) || 
    (l_bound + 1e-10 < s->second && s->second + 1e-10 < r_bound)
  );
};


trajectory::trajectory(double t, double ep, double lp, double v): 
  version(0), entry_time(t), leave_time(t), entry_position(ep), 
  leave_position(lp), entry_velocity(v), 
  leave_velocity(v), sub_trajs(), front_traj_ver(0), front_traj(nullptr) {}

std::optional<trajectory>
trajectory::place_on_top(trajectory t, double ring_len) const {
  if (sub_trajs.empty() || t.sub_trajs.empty()) {
    EXIT("[ERROR] place_on_top\ngot empty list");
  }

  if (t.leave_position - entry_position < -1e-10) {
    t.entry_position += ring_len;
    t.leave_position += ring_len;
    for (auto& st: t.sub_trajs) {
      st.entry_position += ring_len;
      st.leave_position += ring_len;
    }
  }
  t.entry_time += TIME_GAP;
  t.leave_time += TIME_GAP;
  for (auto& st: t.sub_trajs) {
    st.entry_time += TIME_GAP;
    st.leave_time += TIME_GAP;
  }

  trajectory s = *this;
  auto s_it = s.sub_trajs.begin();
  if (s.entry_position == t.entry_position) {
    for (; s_it != s.sub_trajs.end(); ++s_it) {
      auto iter = std::ranges::find_if(t.sub_trajs, [&s_it](auto& st) {
        return s_it->on_top_of(st);
      });
      if (iter != t.sub_trajs.end()) break;
    }
    if (s_it == s.sub_trajs.end()) return s;
  }
  else {
    s_it = s.sub_trajs.end() - 1;
  }

  for (; s_it >= s.sub_trajs.begin(); --s_it) {
    if (s_it->acc == MIN_A) continue;
    for (auto t_it = t.sub_trajs.begin(); t_it != t.sub_trajs.end(); ++t_it) {
      if (t_it->leave_position + 1e-10 < s_it->entry_position) continue;

      auto solutions = tangent_solver(
        s_it->entry_time, s_it->entry_position, s_it->entry_velocity, s_it->acc, 
        t_it->entry_time, t_it->entry_position, t_it->entry_velocity, t_it->acc
      );
      if (solutions) {
        auto& [sol1, sol2] = solutions.value();
        std::optional<std::pair<double, double>> sol;
        if (
          s_it->entry_time <= sol1.first && 
          sol1.first <= sol1.second && 
          (sol1.second <= t_it->leave_time || t_it == t.sub_trajs.end() - 1)
        ) {
          sol = sol1;
        }
        else if (
          s_it->entry_time <= sol2.first && 
          sol2.first <= sol2.second && 
          (sol2.second <= t_it->leave_time || t_it == t.sub_trajs.end() - 1)
        ) {
          sol = sol2;
        }

        if (sol) {
          auto [t_star, t_tang] = sol.value();
          double tmp_acc = s_it->acc;

          trajectory result(s.entry_time, s.entry_position, t.leave_position, s.entry_velocity);
          for (auto it = s.sub_trajs.begin(); it < s_it; ++it) {
            result.push_sub_traj(it->leave_time, it->acc);
          }

          if (t_star > t_it->leave_time) {
            result.push_sub_traj(-1, tmp_acc);
          }
          else {
            result.push_sub_traj(t_star, tmp_acc);
            if (t_tang > t_it->leave_time) {
              result.push_sub_traj(-1, MIN_A);
            }
            else {
              result.push_sub_traj(t_tang, MIN_A);
              result.push_sub_traj(t_it->leave_time, t_it->acc);
            }
          }

          for (auto it = t_it + 1; it != t.sub_trajs.end(); it++) {
            result.push_sub_traj(it->leave_time, it->acc);
          }
          return result;
        }
      }
    }
  }

  return std::nullopt;
}

void trajectory::split(split_iter iter, const split_iter& end) const {
  auto st_iter = sub_trajs.cbegin();
  iter->second.entry_time = entry_time;
  iter->second.entry_velocity = entry_velocity;
  iter->second.clear_trajs();
  iter->second.push_sub_traj(-1, st_iter->acc);

  while (true) {
    if (iter->second.leave_time > st_iter->leave_time + 1e-10) {
      iter->second.sub_trajs.pop_back();
      iter->second.push_sub_traj(st_iter->leave_time, st_iter->acc);
      ++st_iter;
      if (st_iter == sub_trajs.cend()) return;
      iter->second.push_sub_traj(-1, st_iter->acc);
    }
    else {
      if (fabs(iter->second.leave_time - st_iter->leave_time) <= 1e-10) {
        ++st_iter;
        if (st_iter == sub_trajs.cend()) return;
      }
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
    if (acc == 0) {
      if (ev == 0) {
        throw std::logic_error("trajectory::push_sub_traj\nthis should not happen");
      }
      end_time = et + (leave_position - ex) / ev;      
    }
    else {
      double delta = ev*ev + 2 * acc * (leave_position - ex);
      if (delta <= 1e-10) delta = 0; // if delta < 0, decelerate until v = 0
      end_time = et + (sqrt(delta) - ev) / acc;
    }
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
