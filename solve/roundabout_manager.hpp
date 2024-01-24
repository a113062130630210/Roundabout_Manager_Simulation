#pragma once

#include "roundabout.hpp"
#include "scheduling_table.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  struct read_data {
    int id, entry, exit;
    double arr, vel;
  };

  roundabout_manager();

  void load_input(const std::string&);
  void solve();
  void print_result(const std::string&, const std::string&) const;

private:
  roundabout _roundabout;
  std::vector<vehicle> _vehicles;
  scheduling_table _table;
  bool _solved;

  void schedule(int, modular<int>);
};
