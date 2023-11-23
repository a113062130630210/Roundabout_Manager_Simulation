#include "roundabout.hpp"

section::section(int i, double l, double p)
  : index(i), length(l), position(p), unscheduled(std::vector<vehicle>()), scheduled(std::vector<trajectory>()) {}

bool section::unscheduled_before(double time) const {
  return unscheduled.size() != 0 && unscheduled[0].arrival_time < time;
}

roundabout::roundabout(const int c, const std::vector<double>& l)
  : _section_count(c), _sections(std::vector<section>()) {
  double s = 0;
  for (int i = 0; i < c; i++) {
    _sections.push_back(section(i, l[i], s));
    s += l[i];
  }
}

int roundabout::section_count() const {
  return _section_count;
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
