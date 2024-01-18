#include <algorithm>
#include <fstream>

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
    _scheduling_table[*v.entry].emplace_back(v.index, v.arrival_time, false, true);
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
      schedule(v.index, v.entry);
      for (auto& u: _vehicles) {
        if (!u.trajs.empty()) {
          for (auto& [sec_id, t]: u.trajs) {
            if (t.front_traj && t.front_traj->version != t.front_traj_ver) {
              schedule(u.index, { M, sec_id });
            }
          }
        }
      }
    }
  }

  _solved = true;
  std::cout << "Solved." << std::endl;
}

trajectory roundabout_manager::schedule(int index, modular<int> target) {
  vehicle& veh      = _vehicles[index];
  double   cur_time = veh.arrival_time;
  auto&    trajs    = veh.trajs;

  do {
    insert_scheduling_table(*target, index, cur_time, false);

    std::cout << "Scheduling vehicle ";
    std::cout << veh.index << ", section " << target << std::endl;
    do {
      int front = get_unscheduled_front(*target, cur_time);
      if (front == -1) break;
      schedule(front, target);
    } while (true);
    target = veh.progress;

    // TODO: simplify logic
    if (target == veh.exit) break;

    section& cur_sec = _roundabout.section_at(*target);
    trajectory traj = veh.max_velocity(cur_sec.length);

    int front = get_nearest_front(*target, cur_time);
    if (front != -1) {
      const trajectory& top_traj = _vehicles[front].get_traj(*target);
      if (!traj.place_on_top(top_traj)) {
        trajs.push_back({ *target, traj });

        double length = _roundabout.total_length();
        auto sec_id = target - 1;
        auto prev = trajs.rbegin() + 1;
        for (; prev != trajs.rend(); ++prev, sec_id -= 1) {
          std::cout << "avoid front \n" << prev->second << top_traj;
          if (auto res = prev->second.avoid_front(top_traj, length)) {
            res->split(prev.base(), trajs.end());
            goto TEMP;
          }
        }
        if (prev == trajs.rend()) {
          // for (auto& v: _vehicles) {
          //   for (auto& [_, t]: v.trajs) {
          //     std::cout << t;
          //   }
          //   std::cout << std::endl;
          // }
          EXIT("roundabout_manager::schedule\nok this happened");
        }
      }
    }

    trajs.push_back({ *target, traj });

TEMP:
    update_scheduling_table(*target, index, cur_time, true);
    double prev_entry = cur_time;
    for (auto& [idx, entry_time, scheduled, is_entry]: _scheduling_table[*target]) {
      if (scheduled || !is_entry) continue;
      double safe_time = prev_entry + TIME_GAP;
      auto& v = _vehicles[idx];
      if (v.arrival_time > safe_time) break;
      prev_entry = v.arrival_time = entry_time = safe_time;
    }

    target += 1;
    veh.progress += 1;
    veh.arrival_time = cur_time = traj.leave_time;
    veh.current_position = _roundabout.position_of(*target);
    veh.init_velocity = traj.leave_velocity;
  } while (target != veh.exit);

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

    vehicle v(id, { M, entry }, { M, exit }, arrival, _roundabout.position_of(entry), vel);
    v.trajs.reserve(*(v.exit - v.entry));
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