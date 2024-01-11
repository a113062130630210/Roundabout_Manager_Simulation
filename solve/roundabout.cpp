#include <numeric>

#include "roundabout.hpp"

section::section(int i, double l, double p): 
  index(i), length(l), position(p) {}

roundabout::roundabout() {
  roundabout(0, {});
}

roundabout::roundabout
(const int c, const std::vector<double>& l): _section_count(c) {
  _total_length = 0;
  for (int i = 0; i < c; i++) {
    _sections.push_back(section(i, l[i], _total_length));
    _total_length += l[i];
  }
}

roundabout::roundabout
(const int c, std::vector<double>&& l): _section_count(c) {
  _total_length = 0;
  for (int i = 0; i < c; i++) {
    _sections.push_back(section(i, l[i], _total_length));
    _total_length += l[i];
  }
}

int roundabout::section_count() const {
  return _section_count;
}

double roundabout::total_length() const {
  return _total_length;
}

double roundabout::length_of(int i) const {
  if (i < 0 || _section_count < i) {
    i %= _section_count;
    if (i < 0) {
      i += _section_count;
    }
  }
  return _sections[i].length;
}

double roundabout::position_of(int i) const {
  if (i < 0 || _section_count < i) {
    i %= _section_count;
    if (i < 0) {
      i += _section_count;
    }
  }
  return _sections[i].position;
}

section& roundabout::section_at(int i) {
  if (i < 0 || _section_count < i) {
    i %= _section_count;
    if (i < 0) {
      i += _section_count;
    }
  }
  return _sections[i];
}
