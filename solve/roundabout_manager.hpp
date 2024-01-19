#pragma once

#include "roundabout.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  struct read_data {
    int id, entry, exit;
    double arr, vel;
  };

  struct schedule_info {
    schedule_info(int i, double t, bool s, bool ie): 
      index(i), entry_time(t), scheduled(s), is_entry(ie) {}
    int index;
    double entry_time;
    bool scheduled;
    bool is_entry;
  };

  roundabout_manager();

  void load_input(const std::string&);
  void solve();
  void print_result(const std::string&, const std::string&) const;

private:
  roundabout _roundabout;
  std::vector<vehicle> _vehicles;
  std::vector<std::vector<schedule_info>> _scheduling_table;
  bool _solved;

  trajectory schedule(int, modular<int>);
  int get_nearest_front(int, double);
  int get_unscheduled_front(int, double);
  void insert_scheduling_table(int, int, double, bool);
  bool update_scheduling_table(int, int, double, bool);
};
