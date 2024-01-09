#pragma once

#include "vehicle.hpp"

struct section {
  section(int, double, double);

  const int index;
  const double length;
  const double position;
};

class roundabout {
public:
  roundabout();
  roundabout(const int, const std::vector<double>&);
  roundabout(const int, std::vector<double>&&);

  int section_count() const;
  double length_of(int) const;
  double position_of(int) const;
  section& section_at(int);

private:
  int _section_count;
  std::vector<section> _sections;
};