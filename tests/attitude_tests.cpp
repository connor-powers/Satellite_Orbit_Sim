#include <gtest/gtest.h>

#include <iostream>

#include "Satellite.h"
#include "utils.h"


const double epsilon = pow(10.0, -10);

TEST(AttitudeTests, PassivePitchEvolution1) {
  // Without externally-applied torques, the pitch of the satellite
  // with respect to the LVLH frame should progress 2*Pi radians
  // across one full orbit
  double tolerance=pow(10, -10);
  Satellite test_satellite("../tests/elliptical_orbit_attitude_test_1_input.json");
  double test_timestep = 0.1;  // s
  double initial_pitch = test_satellite.get_attitude_val("Pitch");
  double next_timestep = 0;
  bool evolved=false;
  const double initial_true_anomaly=test_satellite.get_orbital_element("True Anomaly");
  double true_anomaly=initial_true_anomaly;
  while ( (true_anomaly < initial_true_anomaly) || (evolved == false) ) {
    std::pair<double, int> new_timestep_and_error_code =
        test_satellite.evolve_RK45(epsilon, test_timestep);
    next_timestep = new_timestep_and_error_code.first;
    int error_code = new_timestep_and_error_code.second;
    test_timestep = next_timestep;
    true_anomaly = test_satellite.get_orbital_element("True Anomaly");
    if ((0 < true_anomaly) && (true_anomaly < initial_true_anomaly)){
      evolved = true;
    }
  }
  double new_calculated_pitch = test_satellite.get_attitude_val("Pitch");

  EXPECT_TRUE(abs(new_calculated_pitch - initial_pitch) > tolerance)
      << "Pitch after an orbit didn't align with expectations within tolerance. "
      "Difference: " << new_calculated_pitch - initial_pitch << "\n";
}
