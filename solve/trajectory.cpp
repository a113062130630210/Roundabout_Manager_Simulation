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

bool subtrajectory::conflict_with(const subtrajectory& st) const {
  {
    double l_x_bound = std::max(entry_position, st.entry_position);
    double r_x_bound = std::min(leave_position, st.leave_position);
    if (l_x_bound > r_x_bound + 1e-10) return false;

    double middle = (l_x_bound + r_x_bound) / 2;
    double d = middle - entry_position;
    double v = entry_velocity;
    double a = acc;
    double delta = v*v + 2*a*d;
    if (fabs(delta) <= 1e-10) delta = 0;
    double s_time = (a == 0 ? d/v : (sqrt(delta) - v) / a) + entry_time;

    d = middle - st.entry_position;
    v = st.entry_velocity;
    a = st.acc;
    delta = v*v + 2*a*d;
    if (fabs(delta) <= 1e-10) delta = 0;
    double t_time = (a == 0 ? d/v : (sqrt(delta) - v) / a) + st.entry_time;

    if (s_time + 1e-10 < t_time) return true;
  }

  // check if `*this` and `traj` intersects
  auto s = quadratic_solver(
       entry_time,    entry_position,    entry_velocity,    acc, 
    st.entry_time, st.entry_position, st.entry_velocity, st.acc
  );
  if (!s) return false;

  // check if any of the intersection pnoints is in the range of trajectories
  double l_t_bound = std::max(entry_time, st.entry_time);
  double r_t_bound = std::min(leave_time, st.leave_time);
  return (
    (l_t_bound + 1e-10 < s->first  && s->first  + 1e-10 < r_t_bound) || 
    (l_t_bound + 1e-10 < s->second && s->second + 1e-10 < r_t_bound)
  );
};


trajectory::trajectory(double t, double ep, double lp, double v): 
  version(0), entry_time(t), leave_time(t), entry_position(ep), 
  leave_position(lp), entry_velocity(v), 
  leave_velocity(v), sub_trajs(), front_traj_ver(0), front_traj(nullptr) {}

std::optional<trajectory>
trajectory::place_on_top(trajectory t, double ring_len) const {
  if (sub_trajs.empty() || t.sub_trajs.empty()) {
    throw std::logic_error("trajectory::place_on_top got empty list");
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
        return s_it->conflict_with(st);
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
    for (auto t_it = t.sub_trajs.end()-1; t_it >= t.sub_trajs.begin(); --t_it) {
      if (t_it->leave_position + 1e-10 < s_it->entry_position) continue;

      std::optional<trajectory> result;
      bool feasible = false;
      if (t_it->acc != MIN_A) {
        if (auto solution = find_tangent(s_it, t_it, t.sub_trajs.end())) {
          feasible = true;
          auto [t_star, t_tang] = solution.value();
          if (t_tang <= t_it->leave_time) {
            trajectory _result(s.entry_time, s.entry_position, t.leave_position, s.entry_velocity);
            for (auto it = s.sub_trajs.begin(); it < s_it; ++it) {
              _result.push_sub_traj(it->leave_time, it->acc);
            }
            _result.push_sub_traj(t_star, s_it->acc);
            _result.push_sub_traj(t_tang, MIN_A);
            _result.push_sub_traj(-1, t_it->acc, t_it->leave_position);

            for (auto it = t_it + 1; it != t.sub_trajs.end(); it++) {
              _result.push_sub_traj(-1, it->acc, it->leave_position);
            }

            if (fabs(_result.leave_time - t.leave_time) <= 1e-10) {
              return _result;
            }
            result = _result;
          }
        }
      }

      if ((t_it->acc == MIN_A || feasible) && (t_it == t.sub_trajs.end() - 1)) {
        if (auto solution = find_point(s_it, t_it)) {
          auto t_star = solution.value();
          trajectory _result(s.entry_time, s.entry_position, t.leave_position, s.entry_velocity);
          for (auto it = s.sub_trajs.begin(); it < s_it; ++it) {
            _result.push_sub_traj(it->leave_time, it->acc);
          }
          _result.push_sub_traj(t_star, s_it->acc);
          _result.push_sub_traj(-1, MIN_A, t_it->leave_position);

          for (auto it = t_it + 1; it != t.sub_trajs.end(); it++) {
            _result.push_sub_traj(-1, it->acc, it->leave_position);
          }

          if (!_result.conflict_with(*t_it)) {
            return _result;
          }
        }
      }

      if (result) {
        return result.value();
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<double, double>> trajectory::find_tangent
(const st_iter& s_it, const st_iter& t_it, const st_iter& t_end) const {
  auto solutions = tangent_solver(
    s_it->entry_time, s_it->entry_position, s_it->entry_velocity, s_it->acc, 
    t_it->entry_time, t_it->entry_position, t_it->entry_velocity, t_it->acc
  );
  if (!solutions) return std::nullopt;

  auto& [sol1, sol2] = solutions.value();
  double a = s_it->acc - MIN_A;
  double b = s_it->entry_velocity - s_it->acc * s_it->entry_time;

  if (
    s_it->entry_time <= sol1.first + 1e-10 && 
    sol1.first <= sol1.second + 1e-10 && 
    sol1.first <= s_it->leave_time + 1e-10 &&
    (sol1.second <= t_it->leave_time + 1e-10 || t_it == t_end - 1) &&
    a * sol1.first + MIN_A * sol1.second + b + 1e-10 >= 0
  ) return sol1;

  if (
    s_it->entry_time <= sol2.first + 1e-10 && 
    sol2.first <= sol2.second + 1e-10 && 
    sol2.first <= s_it->leave_time + 1e-10 &&
    (sol2.second <= t_it->leave_time + 1e-10 || t_it == t_end - 1) &&
    a * sol2.first + MIN_A * sol2.second + b + 1e-10 >= 0
  ) return sol2;

  return std::nullopt;
}

std::optional<double>
trajectory::find_point(const st_iter& s_it, const st_iter& t_it) const {
  auto solutions = point_solver(
    s_it->entry_time, s_it->entry_position, s_it->entry_velocity, s_it->acc, 
    t_it->leave_time, t_it->leave_position
  );
  if (!solutions) return std::nullopt;

  auto& [sol1, sol2] = solutions.value();
  double a1 = s_it->acc;
  double b1 = s_it->entry_velocity - s_it->acc * s_it->entry_time;
  double a2 = s_it->acc - MIN_A;
  double b2 = s_it->entry_velocity - s_it->acc * s_it->entry_time + MIN_A * t_it->leave_time;
  if (
    s_it->entry_time <= sol1 + 1e-10 && sol1 <= s_it->leave_time + 1e-10 &&
    sol1 <= t_it->leave_time + 1e-10 && a1 * sol1 + b1 + 1e-10 >= 0 && a2 * sol1 + b2 + 1e-10 >= 0
  ) return sol1;
  if (
    s_it->entry_time <= sol2 + 1e-10 && sol2 <= s_it->leave_time + 1e-10 &&
    sol2 <= t_it->leave_time + 1e-10 && a1 * sol2 + b1 + 1e-10 >= 0 && a2 * sol2 + b2 + 1e-10 >= 0
  ) return sol2;
  return std::nullopt;
}

// TODO: fix inconsistent shifting
bool trajectory::conflict_with(trajectory t) const {
  t.entry_time += TIME_GAP;
  t.leave_time += TIME_GAP;
  for (auto& st: t.sub_trajs) {
    st.entry_time += TIME_GAP;
    st.leave_time += TIME_GAP;
  }

  auto s_it = sub_trajs.begin();
  auto t_it = t.sub_trajs.begin();
  while (s_it != sub_trajs.end() && t_it != t.sub_trajs.end()) {
    if (s_it->conflict_with(*t_it)) return true;
    bool s_step = false, t_step = false;
    if (s_it->leave_position <= t_it->leave_position + 1e-10) s_step = true;
    if (t_it->leave_position <= s_it->leave_position + 1e-10) t_step = true;
    if (s_step) ++s_it;
    if (t_step) ++t_it;
  }
  return false;
}

bool trajectory::conflict_with(subtrajectory st) const {
  for (auto& s_st: sub_trajs) {
    if (s_st.conflict_with(st)) return true;
  }
  return false;
}

void trajectory::split(t_iter iter, const t_iter& end) const {
  auto st_iter = sub_trajs.cbegin();
  iter->entry_time = entry_time;
  iter->entry_velocity = entry_velocity;
  iter->clear_trajs();
  iter->push_sub_traj(-1, st_iter->acc, false);

  while (true) {
    if (iter->leave_time > st_iter->leave_time + 1e-10) {
      iter->sub_trajs.pop_back();
      iter->push_sub_traj(st_iter->leave_time, st_iter->acc);
      ++st_iter;
      if (st_iter == sub_trajs.cend()) return;
      iter->push_sub_traj(-1, st_iter->acc, false);
    }
    else {
      if (fabs(iter->leave_time - st_iter->leave_time) <= 1e-10) {
        ++st_iter;
        if (st_iter == sub_trajs.cend()) return;
      }
      ++iter;
      if (iter == end) return;
      iter->entry_time = (iter-1)->leave_time;
      iter->entry_velocity = (iter-1)->leave_velocity;
      iter->clear_trajs();
      iter->push_sub_traj(-1, st_iter->acc, false);
    }
  }

  throw std::logic_error("trajectory::split this should not happen");
}

// TODO: remove vel_check
trajectory& trajectory::push_sub_traj
(double end_time, double acc, bool vel_check) {
  return push_sub_traj(end_time, acc, leave_position, vel_check);
}

trajectory& trajectory::push_sub_traj
(double end_time, double acc, double lp, bool vel_check) {
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
        throw std::logic_error("trajectory::push_sub_traj this should not happen");
      }
      end_time = et + (lp - ex) / ev;      
    }
    else {
      double delta = ev*ev + 2 * acc * (lp - ex);
      if (delta <= 1e-10) delta = 0; // if delta < 0, decelerate until v = 0
      end_time = et + (sqrt(delta) - ev) / acc;
    }
  }

  if (fabs(end_time - et) <= 1e-10) return *this;
  if (end_time < et) {
    throw std::logic_error("trajectory::push_sub_traj infeasible end time");
  }

  double t = end_time - et;
  double lx = ex + ev * t + acc * t * t / 2;
  double lv = ev + acc * t;

  if (lx > leave_position + 1e-10) {
    throw std::logic_error("trajectory::push_sub_traj position out of range");
  }
  if (vel_check && (lv > MAX_V + 1e-10)) {
    throw std::logic_error("trajectory::push_sub_traj velocity out of range");
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
