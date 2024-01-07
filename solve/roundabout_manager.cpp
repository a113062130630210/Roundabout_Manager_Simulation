#include <algorithm>
#include <fstream>
#include <stack>

#include "constants.hpp"
#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(): _solved(false) {}

void roundabout_manager::load_input() {
  std::cout << "No input file is provided, using default settings." << std::endl;

  _roundabout = roundabout(8, { 10, 10, 10, 10, 10, 10, 10, 10 });

  _vehicles.push_back(vehicle({ 0, 4, 3, 4.5, 0 }));
  _vehicles.push_back(vehicle({ 1, 0, 7, 0, 0 }));

  for (auto& v: _vehicles) {
    v.current_position = _roundabout.position_of(v.entry);
  }
}

void roundabout_manager::load_input(const std::string&) {
  // TODO: load input from file
  std::cout << "Input file is temporarily ignored." << std::endl;
  load_input();
}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  const int M = _roundabout.section_count();
  const int N = _vehicles.size();

  std::sort(_vehicles.begin(), _vehicles.end(), [](auto& a, auto& b) {
    return a.arrival_time < b.arrival_time;
  });

  for (int i = 0; i < N; i++) {
    _vehicles[i].index = i;
  }

  _scheduling_table.reserve(M);
  for (int i = 0; i < M; i++) {
    _scheduling_table.emplace_back();
  }
  for (auto& v: _vehicles) {
    _scheduling_table[v.entry].emplace_back(v.index, v.arrival_time, unscheduled);
  }
  for (auto& sec: _scheduling_table) {
    double prev = -TIME_GAP;
    for (auto& [idx, t, __]: sec) {
      if (prev + TIME_GAP > t) {
        _vehicles[idx].arrival_time = t = prev + TIME_GAP;
      }
      prev = t;
    }
  }

  for (auto& v: _vehicles) {
    if (v.trajs.empty()) {
      schedule(v.index);
    }
  }

  _solved = true;
  std::cout << "Solved." << std::endl;
}

trajectory roundabout_manager::schedule(int index) {
  vehicle veh      = _vehicles[index];
  int     sec_id   = veh.entry;
  double  cur_time = veh.arrival_time;
  std::vector<trajectory> trajs;

  do {
    std::cout << "Scheduling vehicle ";
    std::cout << veh.index << ", section " << sec_id << std::endl;

    do {
      int front = get_unscheduled_front(sec_id, cur_time);
      if (front == -1) break;
      schedule(front);
    } while (true);

    section& cur_sec = _roundabout.section_at(sec_id);
    trajectory traj = veh.max_velocity(cur_sec.length);

    int front = get_nearest_front(sec_id, cur_time);
    if (front != -1) {
      const trajectory& top_traj = _vehicles[front].trajs.at(sec_id);
      if (!traj.place_on_top(top_traj)) {
        std::stack<trajectory> wayback_trajs;

        for (auto prev = trajs.rbegin(); prev != trajs.rend(); ++prev) {
          if (prev->avoid_front(top_traj)) break;
          trajs.pop_back();
          wayback_trajs.push(*prev);
        }

        double entry_time = trajs.back().leave_time;
        double entry_velocity = trajs.back().leave_velocity;
        int sid = sec_id - wayback_trajs.size();
        sid = sid >= 0 ? sid : sid + _roundabout.section_count();

        while (!wayback_trajs.empty()) {
          trajectory& t = wayback_trajs.top();
          t.sub_trajs.clear();
          t.entry_time = entry_time;
          t.entry_velocity = entry_velocity;
          t.push_sub_traj(-1, MIN_A);

          trajs.push_back(t);
          _vehicles[index].insert_trajectory(sid, t);
          update_scheduling_table(sid, index, entry_time, scheduling);

          wayback_trajs.pop();
          entry_time = t.leave_time;
          entry_velocity = t.leave_velocity;
          sid = sid == _roundabout.section_count() - 1 ? 0 : sid + 1;
        }

        veh.arrival_time = entry_time;
        veh.init_velocity = entry_velocity;
        traj = veh.max_velocity(cur_sec.length);
        if (!traj.place_on_top(top_traj)) {
          EXIT(
            "[ERROR] roundabout_manager::schedule\n"
            "this should not happen"
          );
        }
      }
    }

    trajs.push_back(traj);
    _vehicles[index].insert_trajectory(sec_id, traj);
    update_scheduling_table(sec_id, index, cur_time, scheduling);

    sec_id   = sec_id == _roundabout.section_count() - 1 ? 0 : sec_id + 1;
    veh.arrival_time = cur_time = traj.leave_time;
    veh.current_position = _roundabout.position_of(sec_id);
    veh.init_velocity = traj.leave_velocity;
    insert_scheduling_table(sec_id, index, cur_time, unscheduled);
  } while (sec_id != veh.exit);

  sec_id = veh.entry;
  for (auto& t: trajs) {
    // TODO: make sure section::scheduled is sorted by arrival time
    update_scheduling_table(sec_id, index, t.entry_time, scheduled);
    _vehicles[index].insert_trajectory(sec_id, t);

    // pushes all vehicles in the same section upward
    // to avoid violating safety constraint
    double prev_entry = t.entry_time;
    for (auto& [idx, entry_time, status]: _scheduling_table[sec_id]) {
      if (status != unscheduled) continue;
      double safe_time = prev_entry + TIME_GAP;
      auto& v = _vehicles[idx];
      if (v.arrival_time > safe_time) break;
      prev_entry = v.arrival_time = entry_time = safe_time;
    }

    sec_id = sec_id == _roundabout.section_count() - 1 ? 0 : sec_id + 1;
  }

  return trajs[0];
}

void roundabout_manager::insert_scheduling_table
(int sec_id, int veh_index, double entry_time, schedule_status status) {
  auto& sec = _scheduling_table[sec_id];
  auto it = std::find_if(sec.begin(), sec.end(), [entry_time](schedule_info& i) {
    return i.entry_time >= entry_time;
  });
  sec.insert(it, { veh_index, entry_time, status });
}

bool roundabout_manager::update_scheduling_table
(int sec_id, int veh_index, double entry_time, schedule_status status) {
  auto& sec = _scheduling_table[sec_id];
  auto it = std::find_if(sec.begin(), sec.end(), [veh_index](schedule_info& i) {
    return i.index >= veh_index;
  });
  
  if (it == sec.end()) return false;
  it->entry_time = entry_time;
  it->status = status;
  return true;
}

int roundabout_manager::get_nearest_front(int sec_id, double entry_time) {
  const int N = _scheduling_table[sec_id].size();
  for (int i = N - 1; i >= 0; i--) {
    auto& [idx, t, s] = _scheduling_table[sec_id][i];
    if (s == unscheduled) continue;
    if (t < entry_time) return idx;
  }
  return -1;
}

int roundabout_manager::get_unscheduled_front(int sec_id, double entry_time) {
  if (_scheduling_table[sec_id].size() == 0) return -1;
  for (auto& [idx, t, s]: _scheduling_table[sec_id]) {
    if (t >= entry_time) return -1;
    if (s == unscheduled) return idx;
  }
  return -1;
}

void roundabout_manager::print_result
(const std::string& latex_name, const std::string& format_name) const {
  if (!_solved) {
    EXIT("The problem haven't been solved yet.");
  }

  std::ofstream latex_file, format_file;
  latex_file.open(latex_name);
  format_file.open(format_name);

  format_file << _vehicles.size() << std::endl;
  for (auto& v: _vehicles) {
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