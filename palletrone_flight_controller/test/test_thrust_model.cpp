#include "palletrone_flight_controller/thrust_model.hpp"

#include "gtest/gtest.h"

namespace {
using palletrone_flight_controller::PwmCalibration;
using palletrone_flight_controller::thrust_to_pwm_us;

double thrust_at_pwm(double pwm, const PwmCalibration &calibration) {
  return calibration.quadratic_n_per_us2 * pwm * pwm +
    calibration.linear_n_per_us * pwm + calibration.constant_n;
}

TEST(ThrustModel, InvertsFirmwareQuadraticWithinUnsaturatedRange) {
  const PwmCalibration calibration;
  EXPECT_NEAR(
    thrust_to_pwm_us(thrust_at_pwm(1500.0, calibration), calibration),
    1500.0, 1e-9);
}

TEST(ThrustModel, ReproducesFirmwareNormalizedSafetyLimit) {
  const PwmCalibration calibration;
  EXPECT_NEAR(
    thrust_to_pwm_us(thrust_at_pwm(1900.0, calibration), calibration),
    1740.0, 1e-9);
}
}  // namespace
