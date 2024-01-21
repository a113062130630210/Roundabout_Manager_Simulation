#include <algorithm>
#include <fstream>
#include <iomanip>

#include "constants.hpp"
#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(): _solved(false) {}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  const int M = _roundabout.section_count();
  

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
  double   cur_time = _table.get(*target, index).entry_time;
  double   length   = _roundabout.total_length();
  auto&    trajs    = veh.trajs;

  do {
    _table.insert(*target, index, cur_time, false);

    std::cout << "[VEH " << veh.index << ", SEC " << target << "]" << std::endl;
    do {
      auto front = _table.get_unscheduled_front(*target, cur_time);
      if (front.index == -1) break;
      if (front.entry_time + 0.3 >= cur_time) {
        _table.push(*target, cur_time);
        break;
      }
      schedule(front.index, target);
      cur_time = _table.get(*target, index).entry_time;
    } while (true);

    // TODO: simplify logic
    if (veh.progress == veh.exit) break;

    target = veh.progress;
    veh.entry_time = cur_time;
    _table.update(*target, index, cur_time, true);

    double sec_length = _roundabout.length_of(*target);
    trajectory traj = veh.max_velocity(sec_length);
    trajs.emplace_back(*target, traj);

    auto front = _table.get_nearest_front(*target, cur_time);
    if (front != -1) {
      const auto& top_traj = _vehicles[front].get_traj(*target);
      auto iter = trajs.rbegin();
      for (; iter != trajs.rend(); ++iter) {
        if (auto res = iter->second.place_on_top(top_traj, length)) {
          auto f_iter = iter.base() - 1;
          auto sec_id = target - (int)(std::distance(f_iter, trajs.end()) - 1);
          res->split(f_iter, trajs.end());
          std::for_each(f_iter, trajs.end(), [&sec_id, &index, this](auto& t) {
            _table.update(*sec_id, index, t.second.entry_time, true);
            sec_id += 1;
          });
          break;
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

    std::cout << "<VEH " << veh.index << ", SEC " << target << ">" << std::endl;

    _table.push(*target, trajs.back().second.entry_time);

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

  _roundabout = roundabout(std::move(sections));

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

  _table.load(M, _vehicles);
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

