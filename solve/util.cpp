#include <cmath>
#include <limits>

#include "constants.hpp"
#include "util.hpp"

// x1 + v1*(t - t1) + a1/2*(t - t1)^2 = x2 + v2*(t - t2) + a2/2*(t - t2)^2
// solve t
std::optional<std::pair<double, double>> quadratic_solver(
  double t1, double x1, double v1, double a1, 
  double t2, double x2, double v2, double a2
) {
  // coefficients of the new parabola
  double A = (a1 - a2) / 2;
  double B = v1 - v2 - a1*t1 + a2*t2;
  double C = x1 - x2 - v1*t1 + v2*t2 + (a1*t1*t1 - a2*t2*t2) / 2;

  // the leading coef is zero, solve it as a linear equation
  if (fabs(A) <= 1e-10) {
    // parallel parabollas are treated as no solution
    if (fabs(B) <= 1e-10) return std::nullopt;

    double R = -C / B;
    return std::make_pair(R, R);
  }

  double delta = B*B - 4*A*C;
  if (delta <= 1e-10) return std::nullopt;

  double X = -B / (2*A);
  double Y = sqrt(delta) / (2*A);
  return std::make_pair(X - Y, X + Y);
}

std::optional<std::pair<std::pair<double, double>, std::pair<double, double>>>
tangent_solver(
  double t1, double x1, double v1, double a1, 
  double t2, double x2, double v2, double a2
) {
  double term = v1 - v2 - a1*t1 + a2*t2;

  if (MIN_A == a2) {
    if (a1 == a2) {
      if (term > 0) return std::nullopt;
      return std::make_pair(std::make_pair(t1, t1), std::make_pair(t1, t1));
    }

    double T = -term / (a1 - a2);
    return std::make_pair(std::make_pair(T, T), std::make_pair(T, T));
  }

  double A = (a1 - MIN_A) * (a1 - a2);
  double B = 2 * (a1 - MIN_A) * term;
  double C = term*term +
    (a2 - MIN_A) * (2 * (x1 - x2 - v1*t1 + v2*t2) + a1*t1*t1 - a2*t2*t2);

  double T1, T2;
  if (fabs(A) <= 1e-10) { // a1 == MIN_A || a1 == a2
    if (fabs(B) <= 1e-10) {
      if (C <= -1e-10) return std::nullopt;
      return std::make_pair(std::make_pair(t1, t1), std::make_pair(t1, t1));
    }

    T1 = T2 = -C / B;
  }
  else {
    double delta = B*B - 4*A*C;
    if (delta <= 1e-10) return std::nullopt;

    double X = -B / (2*A);
    double Y = sqrt(delta) / (2*A);
    T1 = X - Y;
    T2 = X + Y;
  }

  double P1 = ((MIN_A - a1) * T1 - term) / (MIN_A - a2);
  double P2 = ((MIN_A - a1) * T2 - term) / (MIN_A - a2);
  
  return std::make_pair(std::make_pair(T1, P1), std::make_pair(T2, P2));
}

