#include "roundabout_manager.hpp"

roundabout_manager::roundabout_manager(roundabout& r, std::vector<vehicle>& vs): _r(r), _vs(vs) {
  for (auto& v: vs) {
    v.current_position = _r.position_of(v.entry);
  }
}

void roundabout_manager::solve() {
  std::cout << "Solving..." << std::endl;

  const int M = _r.section_count();

  std::sort(_vs.begin(), _vs.end(), [](vehicle& a, vehicle& b) {
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

  std::cout << "Solved." << std::endl;
}

trajectory roundabout_manager::schedule_first_vehicle(section& sec) {
  if (!sec.unscheduled.size()) {
    EXIT("[RANGE_ERROR] roundabout_manager::schedule_first_vehicle\nThere is no unscheduled vehicle.");
  }

  int sec_id = sec.index;
  bool is_entry = true;
  vehicle veh = sec.unscheduled.front();
  std::vector<trajectory> stack;


  do {
    std::cout << "Scheduling vehicle " << veh.id << ", section " << sec_id << std::endl;
    int nxt_sec_id = sec_id == _r.section_count() - 1 ? 0 : sec_id + 1;
    const section& cur_sec = _r.section_at(sec_id);
    trajectory traj = veh.max_velocity(cur_sec.length);
    traj.is_entry = is_entry;

    if (cur_sec.scheduled.size()) {
      const trajectory& top_traj = cur_sec.scheduled.back();
      bool success = traj.place_on_top(top_traj);
      if (!success) {
        EXIT("[DEBUG] ok this happens (place_on_top not success)");
      }
    }

    std::optional<trajectory> front_traj = std::nullopt;
    section& nxt_sec = _r.section_at(nxt_sec_id);
    while (nxt_sec.unscheduled_before(traj.leave_time)) {
      front_traj = schedule_first_vehicle(nxt_sec);
    }
    if (front_traj) {
      bool success = traj.avoid_front(front_traj.value());
      if (!success) {
        EXIT("[DEBUG] ok this happens (avoid_front not success)");
      }
    }
    
    _vs[veh.id].trajs.push_back(std::make_pair(sec_id, traj));
    stack.push_back(traj);

    sec_id = nxt_sec_id;
    is_entry = false;

    veh.entry = sec_id;
    veh.arrival_time = traj.leave_time;
    veh.current_position = _r.position_of(sec_id);
    veh.init_velocity = traj.leave_velocity;
  } while (sec_id != veh.exit);

  sec_id = sec.index;
  for (auto& t: stack) {
    // TODO: make sure section::scheduled is sorted by arrival time
    section& tmp_sec = _r.section_at(sec_id);
    tmp_sec.scheduled.push_back(t);

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

  return stack[0];
}