#pragma once

#include "vehicle.hpp"

class scheduling_table {
public:
  struct entry {
    entry(int i, double t, bool s, bool ie): 
      index(i), entry_time(t), scheduled(s), is_entry(ie) {}
    int index;
    double entry_time;
    bool scheduled;
    bool is_entry;
  };

  void load(int, const std::vector<vehicle>&);
  bool insert(int, int, double, bool, bool = false);
  bool update(int, int, double, bool);
  void push(int, double);
  const entry& get(int, int) const;
  int get_nearest_front(int, double);
  const entry& get_unscheduled_front(int, double);

  static const entry null_entry;

private:
  std::vector<std::vector<entry>> _table;
};
