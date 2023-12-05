#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "constants.hpp"

std::optional<std::pair<double, double>> quadratic_solver(double, double, double, double, double, double, double, double);
std::optional<std::pair<std::pair<double, double>, std::pair<double, double>>> tangent_solver(double, double, double, double, double, double, double, double);