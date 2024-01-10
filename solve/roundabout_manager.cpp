#include <algorithm>
#include <fstream>
#include <stack>

#include "constants.hpp"
#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(): _solved(false) {}

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
    _scheduling_table[v.entry].emplace_back(v.index, v.arrival_time, false, true);
  }
  for (auto& sec: _scheduling_table) {
    double prev = -TIME_GAP;
    for (auto& [idx, t, _, __]: sec) {
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
  vehicle& veh      = _vehicles[index];
  double   cur_time = veh.arrival_time;
  auto&    trajs    = veh.trajs;

  do {
    insert_scheduling_table(veh.progress, index, cur_time, false);

    std::cout << "Scheduling vehicle ";
    std::cout << veh.index << ", section " << veh.progress << std::endl;
    do {
      int front = get_unscheduled_front(veh.progress, cur_time);
      if (front == -1) break;
      schedule(front);
    } while (true);

    // TODO: simplify logic
    if (veh.progress == veh.exit) break;

    section& cur_sec = _roundabout.section_at(veh.progress);
    trajectory traj = veh.max_velocity(cur_sec.length);

    int front = get_nearest_front(veh.progress, cur_time);
    if (front != -1) {
      const trajectory& top_traj = _vehicles[front].get_traj(veh.progress);
      if (!traj.place_on_top(top_traj)) {
        std::stack<trajectory> wayback_trajs;

        for (auto prev = trajs.rbegin(); prev != trajs.rend(); ++prev) {
          if (prev->second.avoid_front(top_traj)) break;
          trajs.pop_back();
          wayback_trajs.push(prev->second);
        }

        double entry_time = trajs.back().second.leave_time;
        double entry_velocity = trajs.back().second.leave_velocity;
        int sec_id = veh.progress - wayback_trajs.size();
        sec_id = sec_id >= 0 ? sec_id : sec_id + _roundabout.section_count();

        while (!wayback_trajs.empty()) {
          trajectory& t = wayback_trajs.top();
          t.sub_trajs.clear();
          t.entry_time = entry_time;
          t.entry_velocity = entry_velocity;
          t.push_sub_traj(-1, MIN_A);

          trajs.push_back({ sec_id, t });
          update_scheduling_table(sec_id, index, entry_time, true);

          wayback_trajs.pop();
          entry_time = t.leave_time;
          entry_velocity = t.leave_velocity;
          sec_id = sec_id == _roundabout.section_count() - 1 ? 0 : sec_id + 1;
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

    trajs.push_back({ veh.progress, traj });
    update_scheduling_table(veh.progress, index, cur_time, true);

    veh.progress = veh.progress == _roundabout.section_count() - 1 ? 0 : veh.progress + 1;
    veh.arrival_time = cur_time = traj.leave_time;
    veh.current_position = _roundabout.position_of(veh.progress);
    veh.init_velocity = traj.leave_velocity;
  } while (veh.progress != veh.exit);

  int sec_id = veh.entry;
  for (auto& [_, t]: trajs) {
    // TODO: probably removable
    update_scheduling_table(sec_id, index, t.entry_time, true);

    // pushes all vehicles in the same section upward
    // to avoid violating safety constraint
    double prev_entry = t.entry_time;
    for (auto& [idx, entry_time, scheduled, is_entry]: _scheduling_table[sec_id]) {
      if (scheduled || !is_entry) continue;
      double safe_time = prev_entry + TIME_GAP;
      auto& v = _vehicles[idx];
      if (v.arrival_time > safe_time) break;
      prev_entry = v.arrival_time = entry_time = safe_time;
    }

    sec_id = sec_id == _roundabout.section_count() - 1 ? 0 : sec_id + 1;
  }

  return trajs[0].second;
}

void roundabout_manager::load_input(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    EXIT("Failed to open input file.");
  }

  int N, M;
  std::vector<double> sections;

  file >> N >> M;
  for (int i = 0; i < M; i++) {
    double t;
    file >> t;
    sections.push_back(t);
  }

  _roundabout = roundabout(M, std::move(sections));

  for (int i = 0; i < N; i++) {
    int id, entry, exit;
    double arrival, vel;
    file >> id >> entry >> exit >> arrival >> vel;

    vehicle v = { id, entry, exit, arrival, vel };
    v.current_position = _roundabout.position_of(entry);
    _vehicles.push_back(std::move(v));
  }
}

void roundabout_manager::print_result
(const std::string& latex_name, const std::string& format_name) const {
  if (!_solved) {
    EXIT("The problem haven't been solved yet.");
  }

  std::ofstream latex_file(latex_name);
  std::ofstream format_file(format_name);

  if (!latex_file.is_open()) {
    EXIT("Failed to open latex file.");
  }
  if (!format_file.is_open()) {
    EXIT("Failed to open format file.");
  }

  for (auto& v: _vehicles) {
    format_file << v.id << " " << v.trajs.back().second.leave_time;
    for (auto& [_, traj]: v.trajs) {
      latex_file << traj;

      for (auto& st: traj.sub_trajs) {
        format_file << " " << st.entry_time << " " << st.acc;
      }
    }
    latex_file << std::endl;
    format_file << std::endl;
  }
}

void roundabout_manager::insert_scheduling_table
(int sec_id, int veh_index, double entry_time, bool scheduled) {
  auto& sec = _scheduling_table[sec_id];

  // insertion fail if the entry already exists
  auto it = std::find_if(sec.begin(), sec.end(), [veh_index](schedule_info& i) {
    return i.index == veh_index;
  });
  if (it != sec.end()) return;

  it = std::find_if(sec.begin(), sec.end(), [entry_time](schedule_info& i) {
    return i.entry_time >= entry_time;
  });

  sec.insert(it, { veh_index, entry_time, scheduled, false });
}

bool roundabout_manager::update_scheduling_table
(int sec_id, int veh_index, double entry_time, bool scheduled) {
  auto& sec = _scheduling_table[sec_id];
  auto it = std::find_if(sec.begin(), sec.end(), [veh_index](schedule_info& i) {
    return i.index == veh_index;
  });
  
  if (it == sec.end()) return false;
  it->entry_time = entry_time;
  it->scheduled = scheduled;
  return true;
}

int roundabout_manager::get_nearest_front(int sec_id, double entry_time) {
  const int N = _scheduling_table[sec_id].size();
  for (int i = N - 1; i >= 0; i--) {
    auto& [idx, t, s, _] = _scheduling_table[sec_id][i];
    if (!s) continue;
    if (t < entry_time) return idx;
  }
  return -1;
}

int roundabout_manager::get_unscheduled_front(int sec_id, double entry_time) {
  if (_scheduling_table[sec_id].size() == 0) return -1;
  for (auto& [idx, t, s, _]: _scheduling_table[sec_id]) {
    if (t >= entry_time) return -1;
    if (!s) return idx;
  }
  return -1;
}