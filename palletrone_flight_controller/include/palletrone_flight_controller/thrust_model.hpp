#pragma once

#include <algorithm>
#include <cmath>

namespace palletrone_flight_controller {

struct PwmCalibration {
  double quadratic_n_per_us2{2.0962e-5};
  double linear_n_per_us{0.0085};
  double constant_n{-36.0347};
  double minimum_us{1100.0};
  double maximum_us{1900.0};
  double normalized_limit{0.8};
};

/// Invert the thrust calibration and reproduce the PX4 normalized-output limit.
inline double thrust_to_pwm_us(double thrust_n, const PwmCalibration &calibration) {
  const double discriminant =
    calibration.linear_n_per_us * calibration.linear_n_per_us -
    4.0 * calibration.quadratic_n_per_us2 * (calibration.constant_n - thrust_n);
  if (discriminant < 0.0) return calibration.minimum_us;
  const double raw_pwm = (
    -calibration.linear_n_per_us + std::sqrt(discriminant)) /
    (2.0 * calibration.quadratic_n_per_us2);
  const double normalized = std::clamp(
    (raw_pwm - calibration.minimum_us) /
    (calibration.maximum_us - calibration.minimum_us),
    0.0, calibration.normalized_limit);
  return calibration.minimum_us + normalized *
    (calibration.maximum_us - calibration.minimum_us);
}

}  // namespace palletrone_flight_controller
