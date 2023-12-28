#include <algorithm>
#include <fstream>
#include <iostream>
#include <stack>
#include <optional>

#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager
(roundabout& r, std::vector<vehicle>& vs): _r(r), _vs(vs), _solved(false) {
  for (auto& v: _vs) {
    v.current_position = _r.position_of(v.entry);
  }
}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  const int M = _r.section_count();

  std::sort(_vs.begin(), _vs.end(), [](auto& a, auto& b) {
    return a.arrival_time < b.arrival_time;
  });

  for (auto& v: _vs) {
    _r.section_at(v.entry).unscheduled.push_back(v);
  }

  for (int i = 0; i < M; i++) {
  // for (int i = M-1; i >= 0; i--) {
    section& sec = _r.section_at(i);
    while (sec.unscheduled.size() > 0) {
      schedule_first_vehicle(sec);
    }
  }

  _solved = true;
  std::cout << "Solved." << std::endl;
}

void roundabout_manager::print_result() {
  if (!_solved) {
    std::cout << "The problem haven't been solved yet." << std::endl;
    solve();
  }

  std::ofstream latex_file, format_file;
  latex_file.open("trajectories.txt");
  format_file.open("format.txt");

  format_file << _vs.size() << std::endl;
  for (auto& v: _vs) {
    for (auto& [_, traj]: v.trajs) {
      latex_file << traj;

      for (auto& st: traj.sub_trajs) {
        format_file << st.entry_time << " " << st.acc << " ";
      }
    }
    latex_file << std::endl;
    format_file << std::endl;
  }
}

trajectory roundabout_manager::schedule_first_vehicle(section& sec) {
  if (!sec.unscheduled.size()) {
    EXIT(
      "[ERROR] roundabout_manager::schedule_first_vehicle\n"
      "There is no unscheduled vehicle."
    );
  }

  int sec_id = sec.index;
  bool is_entry = true;
  vehicle veh = sec.unscheduled.front();
  std::vector<trajectory> trajs;

  do {
    std::cout << "Scheduling vehicle ";
    std::cout << veh.id << ", section " << sec_id << std::endl;

    int nxt_sec_id = sec_id == _r.section_count() - 1 ? 0 : sec_id + 1;
    section& cur_sec = _r.section_at(sec_id);
    trajectory traj = veh.max_velocity(cur_sec.length);
    traj.is_entry = is_entry;

    if (cur_sec.unscheduled_before(traj.entry_time)) {
      schedule_first_vehicle(cur_sec);
    }

    if (!cur_sec.scheduled.empty()) {
      const trajectory& top_traj = cur_sec.scheduled.back();
      if (!traj.place_on_top(top_traj)) {
        std::stack<trajectory> wayback_trajs;

        for (auto prev = trajs.rbegin(); prev != trajs.rend(); ++prev) {
          if (prev->avoid_front(top_traj)) break;
          trajs.pop_back();
          wayback_trajs.push(*prev);
        }

        double entry_time = trajs.back().leave_time;
        double entry_velocity = trajs.back().leave_velocity;
        while (!wayback_trajs.empty()) {
          trajectory& t = wayback_trajs.top();
          t.sub_trajs.clear();
          t.entry_time = entry_time;
          t.entry_velocity = entry_velocity;
          t.push_sub_traj(-1, MIN_A);

          trajs.push_back(t);

          wayback_trajs.pop();
          entry_time = t.leave_time;
          entry_velocity = t.leave_velocity;
        }

        veh.arrival_time = entry_time;
        veh.init_velocity = entry_velocity;
        traj = veh.max_velocity(cur_sec.length);
        if (!traj.place_on_top(top_traj)) {
          EXIT(
            "[ERROR] roundabout_manager::schedule_first_vehicle\n"
            "this should not happen"
          );
        }
      }
    }

    trajs.push_back(traj);

    sec_id = nxt_sec_id;
    is_entry = false;

    veh.arrival_time = traj.leave_time;
    veh.current_position = _r.position_of(sec_id);
    veh.init_velocity = traj.leave_velocity;
  } while (sec_id != veh.exit);

  sec_id = sec.index;
  for (auto& t: trajs) {
    // TODO: make sure section::scheduled is sorted by arrival time
    section& tmp_sec = _r.section_at(sec_id);
    tmp_sec.scheduled.push_back(t);
    _vs[veh.id].trajs.push_back(std::make_pair(sec_id, t));

    // pushes all vehicles in the same section upward
    // to avoid violating safety constraint
    double prev_entry = t.entry_time;
    for (auto& v: tmp_sec.unscheduled) {
      double safe_time = prev_entry + TIME_GAP;
      if (v.arrival_time > safe_time) break;
      prev_entry = v.arrival_time = safe_time;
    }

    sec_id = sec_id == _r.section_count() - 1 ? 0 : sec_id + 1;
  }

  sec.unscheduled.erase(sec.unscheduled.begin());

  return trajs[0];
}