#pragma once

#include "roundabout.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  enum schedule_status {
    unscheduled, scheduling, scheduled
  };

  struct schedule_info {
    schedule_info(int i, double t, schedule_status s): 
      index(i), entry_time(t), status(s) {}
    int index;
    double entry_time;
    schedule_status status;
  };

  roundabout_manager();

  void load_input();
  void load_input(const std::string&);
  void solve();
  void print_result(const std::string&, const std::string&) const;

private:
  roundabout _roundabout;
  std::vector<vehicle> _vehicles;
  std::vector<std::vector<schedule_info>> _scheduling_table;
  bool _solved;

  trajectory schedule(int);
  int get_nearest_front(int, double);
  int get_unscheduled_front(int, double);
  void insert_scheduling_table(int, int, double, schedule_status);
  bool update_scheduling_table(int, int, double, schedule_status);
};
