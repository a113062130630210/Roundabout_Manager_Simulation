#pragma once

#include <vector>

#include "vehicle.hpp"

struct section {
  section(int, double, double);

  bool unscheduled_before(double) const;

  const int index;
  const double length;
  const double position;
  std::vector<vehicle> unscheduled;
  std::vector<trajectory> scheduled;
};

class roundabout {
public:
  roundabout();
  roundabout(const int, const std::vector<double>&);

  int section_count() const;
  double length_of(int) const;
  double position_of(int) const;
  section& section_at(int);

private:
  int _section_count;
  std::vector<section> _sections;
};