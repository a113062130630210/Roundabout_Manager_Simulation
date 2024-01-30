#include <algorithm>
#include <fstream>
#include <iomanip>

#include "constants.hpp"
#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(): _solved(false) {}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  for (auto& v: _vehicles) {
    if (v.progress == v.exit) continue;
    schedule(v.index, v.entry);

    bool detected;
    do {
      detected = false;
      for (auto& u: _vehicles) {
        auto sec_id = u.entry;
        for (auto& t: u.trajs) {
          if (t.front_traj && t.front_traj->version != t.front_traj_ver) {
            if (t.conflict_with(*t.front_traj)) {
              detected = true;
              u.progress = sec_id;
              u.entry_time = t.entry_time;
              u.cur_pos = t.entry_position;
              u.entry_velocity = t.entry_velocity;
              schedule(u.index, sec_id);
              break;
            }

            t.front_traj_ver = t.front_traj->version;
          }
          ++sec_id;
        }
      }
    } while (detected);
  }

  _solved = true;
  std::cout << "Solved." << std::endl;
}

void roundabout_manager::schedule(int index, modular<int> target) {
  vehicle& veh      = _vehicles[index];
  double   cur_time = _table.get(*target, index).entry_time;
  double   length   = _roundabout.total_length();
  auto&    trajs    = veh.trajs;

  do {
    _table.insert(*target, index, cur_time, false);

    std::cout << "[VEH " << index << ", SEC " << target << "]" << std::endl;
    do {
      auto front = _table.get_unscheduled_front(*target, cur_time);
      if (front.index == -1) break;
      // TODO: better yielding criteria
      if (front.entry_time + 0.3 >= cur_time) {
        _table.push(*target, cur_time);
        break;
      }
      schedule(front.index, target);
      cur_time = _table.get(*target, index).entry_time;
    } while (true);

    if (veh.progress == veh.exit) break;

    target = veh.progress;
    _table.update(*target, index, cur_time, true);

    auto traj_iter = veh.get_traj(target);
    if (traj_iter == trajs.end()) {
      throw std::logic_error("roundabout_manager::schedule trajectory not found");
    }

    veh.entry_time = cur_time;
    veh.max_velocity(*traj_iter, _roundabout.length_of(*target));

    int front = _table.get_nearest_front(*target, cur_time);
    if (front != -1) {
      auto front_iter = _vehicles[front].get_traj(target);
      if (front_iter == _vehicles[front].trajs.end()) {
        throw std::logic_error("roundabout_manager::schedule trajectory not found");
      }

      const auto& top_traj = *front_iter;
      auto iter = traj_iter;
      for (; iter >= trajs.begin(); --iter) {
        if (auto res = iter->place_on_top(top_traj, length)) {
          auto sec_id = veh.entry + (int)(iter - trajs.begin());
          res->split(iter, traj_iter + 1);
          std::for_each(iter, traj_iter + 1, [&sec_id, &index, this](auto& t) {
            _table.update(*sec_id, index, t.entry_time, true);
            ++t.version;
            ++sec_id;
          });

          traj_iter->front_traj = &top_traj;
          traj_iter->front_traj_ver = top_traj.version;
          break;
        }
      }
      if (iter < trajs.begin()) {
        // for (auto& v: _vehicles) {
        //   for (auto& t: v.trajs) {
        //     std::cout << t;
        //   }
        //   std::cout << std::endl;
        // }
        throw std::logic_error("roundabout_manager::schedule ok this happened");
      }
    }

    std::cout << "<VEH " << index << ", SEC " << target << ">" << std::endl;

    _table.push(*target, traj_iter->entry_time);

    ++target;
    ++veh.progress;
    veh.entry_time = cur_time = traj_iter->leave_time;
    veh.cur_pos = _roundabout.position_of(*target);
    veh.entry_velocity = traj_iter->leave_velocity;
  } while (target != veh.exit);
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
    format_file << v.id << " " << v.trajs.back().leave_time;
    for (auto& traj: v.trajs) {
      latex_file << traj;

      for (auto& st: traj.sub_trajs) {
        format_file << " " << st.entry_time << " " << st.acc;
      }
    }
    latex_file << std::endl;
    format_file << std::endl;
  }
}

