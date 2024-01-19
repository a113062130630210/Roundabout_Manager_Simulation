#include <algorithm>
#include <fstream>
#include <iomanip>

#include "constants.hpp"
#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(): _solved(false) {}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  const int M = _roundabout.section_count();

  _scheduling_table.reserve(M);
  for (int i = 0; i < M; i++) {
    _scheduling_table.emplace_back();
  }
  for (auto& v: _vehicles) {
    _scheduling_table[*v.entry].emplace_back(v.index, v.entry_time, false, true);
  }
  for (auto& sec: _scheduling_table) {
    double prev = -TIME_GAP;
    for (auto& [idx, t, _, __]: sec) {
      if (prev + TIME_GAP > t) {
        _vehicles[idx].entry_time = t = prev + TIME_GAP;
      }
      prev = t;
    }
  }

  for (auto& v: _vehicles) {
    if (!v.trajs.empty()) continue;
    schedule(v.index, v.entry);
    for (auto& u: _vehicles) {
      if (u.trajs.empty()) continue;
      for (auto& [sec_id, t]: u.trajs) {
        if (t.front_traj && t.front_traj->version != t.front_traj_ver) {
          schedule(u.index, { M, sec_id });
        }
      }
    }
  }

  _solved = true;
  std::cout << "Solved." << std::endl;
}

trajectory roundabout_manager::schedule(int index, modular<int> target) {
  vehicle& veh      = _vehicles[index];
  double   cur_time = veh.entry_time;
  double   length   = _roundabout.total_length();
  auto&    trajs    = veh.trajs;

  do {
    insert_scheduling_table(*target, index, cur_time, false);

    std::cout << "Scheduling vehicle ";
    std::cout << veh.index << ", section " << target << std::endl;
    do {
      int front = get_unscheduled_front(*target, cur_time);
      // if (veh.index == 0 && *target == 0) EXIT("TEMP");
      if (front == -1) break;
      schedule(front, target);
    } while (true);
    target = veh.progress;

    // TODO: simplify logic
    if (target == veh.exit) break;

    double sec_length = _roundabout.length_of(*target);
    trajectory traj = veh.max_velocity(sec_length);
    trajs.push_back({ *target, traj });
    update_scheduling_table(*target, index, cur_time, true);

    int front = get_nearest_front(*target, cur_time);
    if (front != -1) {
      const trajectory& top_traj = _vehicles[front].get_traj(*target);
      auto iter = trajs.rbegin();
      for (; iter != trajs.rend(); ++iter) {
        if (auto res = iter->second.place_on_top(top_traj, length)) {
          auto f_iter = iter.base() - 1;
          res->split(f_iter, trajs.end());
          std::for_each(f_iter, trajs.end(), [&target, &index, this](auto& t) {
            update_scheduling_table(*target, index, t.second.entry_time, true);
          });
          goto TEMP;
        }
      }
      if (iter == trajs.rend()) {
        // for (auto& v: _vehicles) {
        //   for (auto& [_, t]: v.trajs) {
        //     std::cout << t;
        //   }
        //   std::cout << std::endl;
        // }
        EXIT("roundabout_manager::schedule\nok this happened");
      }
    }

TEMP:
    double prev_entry = cur_time;
    for (auto& [idx, entry_time, scheduled, is_entry]: _scheduling_table[*target]) {
      if (scheduled || !is_entry) continue;
      double safe_time = prev_entry + TIME_GAP;
      auto& v = _vehicles[idx];
      if (v.entry_time > safe_time) break;
      prev_entry = v.entry_time = entry_time = safe_time;
    }

    target += 1;
    veh.progress += 1;
    veh.entry_time = cur_time = trajs.back().second.leave_time;
    veh.cur_pos = _roundabout.position_of(*target);
    veh.entry_velocity = trajs.back().second.leave_velocity;
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

  std::vector<read_data> data;
  data.reserve(N);
  for (int i = 0; i < N; i++) {
    int id, entry, exit;
    double arr, vel;
    file >> id >> entry >> exit >> arr >> vel;
    data.emplace_back(id, entry, exit, arr, vel);


  }

  std::sort(data.begin(), data.end(), [](auto& a, auto& b) {
    return a.arr < b.arr;
  });

  int index = 0;
  std::ranges::for_each(data, [this, M, &index](auto& d) {
    double pos = _roundabout.position_of(d.entry);
    vehicle v(d.id, index++, { M, d.entry }, { M, d.exit }, d.arr, pos, d.vel);
    _vehicles.push_back(std::move(v));
  });
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
    if (t < entry_time) {
      return idx;
    }
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