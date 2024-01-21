#include <algorithm>
#include <stdexcept>

#include "constants.hpp"
#include "scheduling_table.hpp"

const scheduling_table::entry scheduling_table::null_entry = { -1, 0, 0, 0 };

void scheduling_table::load(int M, const std::vector<vehicle>& vehicles) {
  _table.clear();
  _table.reserve(M);

  for (int i = 0; i < M; i++) {
    _table.emplace_back();
  }

  std::ranges::for_each(vehicles, [this](const vehicle& v) {
    _table[*v.entry].emplace_back(v.index, v.entry_time, false, true);
  });

  for (auto& sec: _table) {
    double prev = -TIME_GAP;
    for (auto& [_, t, __, ___]: sec) {
      if (prev + TIME_GAP > t) {
        t = prev + TIME_GAP;
      }
      prev = t;
    }
  }
}

bool scheduling_table::insert
(int sec_id, int veh_index, double entry_time, bool scheduled) {
  auto& sec = _table[sec_id];
  auto it = std::ranges::find_if(sec, [veh_index](entry& i) {
    return i.index == veh_index;
  });
  if (it != sec.end()) return false;

  it = std::ranges::find_if(sec, [entry_time](entry& i) {
    return i.entry_time >= entry_time;
  });

  sec.insert(it, { veh_index, entry_time, scheduled, false });
  return true;
}

bool scheduling_table::update
(int sec_id, int veh_index, double entry_time, bool scheduled) {
  auto& sec = _table[sec_id];
  auto it = std::ranges::find_if(sec, [veh_index](entry& i) {
    return i.index == veh_index;
  });
  
  if (it == sec.end()) return false;
  sec.erase(it);
  insert(sec_id, veh_index, entry_time, scheduled);
  return true;
}

void scheduling_table::push(int sec_id, double prev_entry) {
  auto& sec = _table[sec_id];
  for (auto& [idx, entry_time, scheduled, is_entry]: sec) {
    if (scheduled || !is_entry) continue;
    double safe_time = prev_entry + TIME_GAP;
    if (entry_time > safe_time) break;
    prev_entry = entry_time = safe_time;
  }

  std::sort(sec.begin(), sec.end(), [](auto& a, auto& b) {
    return a.entry_time < b.entry_time;
  });
}

const scheduling_table::entry&
scheduling_table::get(int sec_id, int veh_index) const {
  auto& sec = _table[sec_id];
  auto it = std::ranges::find_if(sec, [veh_index](const entry& i) {
    return i.index == veh_index;
  });
  if (it == sec.end()) {
    throw std::range_error("scheduling_table::get index not found");
  }
  return *it;
}

int scheduling_table::get_nearest_front(int sec_id, double entry_time) {
  const int N = _table[sec_id].size();
  for (int i = N - 1; i >= 0; i--) {
    auto& [idx, t, s, _] = _table[sec_id][i];
    if (!s) continue;
    if (t < entry_time) {
      return idx;
    }
  }
  return -1;
}

const scheduling_table::entry&
scheduling_table::get_unscheduled_front(int sec_id, double entry_time) {
  for (auto& entry: _table[sec_id]) {
    if (entry.entry_time >= entry_time) return null_entry;
    if (!entry.scheduled) return entry;
  }
  return null_entry;
}