#pragma once

#include <optional>
#include <utility>

std::optional<std::pair<double, double>>
quadratic_solver
(double, double, double, double, double, double, double, double);

std::optional<std::pair<std::pair<double, double>, std::pair<double, double>>> 
tangent_solver
(double, double, double, double, double, double, double, double);

std::optional<std::pair<double, double>>
point_solver
(double, double, double, double, double, double);

std::pair<double, double>
acc_solver
(double, double, double, double, double, double, double);