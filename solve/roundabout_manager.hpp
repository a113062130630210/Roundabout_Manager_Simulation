#pragma once

#include "roundabout.hpp"
#include "vehicle.hpp"

class roundabout_manager {
public:
  roundabout_manager();

  void load_input();
  void load_input(const std::string&);
  void solve();
  void print_result(const std::string&, const std::string&) const;

private:
  roundabout _r;
  std::vector<vehicle> _vs;
  bool _solved;

  trajectory schedule_first_vehicle(section&);
};
