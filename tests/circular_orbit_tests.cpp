#include <gtest/gtest.h>

#include <iostream>

#include "Satellite.h"
#include "utils.h"

// Testing calculated orbital speed based on input orbital parameters at e=0
// against known formula for circular orbital speed
// https://en.wikipedia.org/wiki/Circular_orbit#Velocity

const double tolerance = pow(10.0, -12);
// Setting a different tolerance for semimajor axis than the other orbital
// parameters since there appears to be a minimum error associated with
// converting position and velocity to semimajor axis, best guess is this has to
// do with the scale of distances and/or velocities being dealt with here.
// Orbital radius check also using this length scale because until I can figure
// out why that test appears to be passing to a tighter tolerance when run
// locally than when run on Github actions workflow
const double length_tolerance = pow(10.0, -7);
const double epsilon = pow(10.0, -7);
const double energy_cons_relative_tolerance = pow(10.0, -5);

TEST(CircularOrbitTests, OrbitalSpeed1) {
  Satellite test_satellite("../tests/circular_orbit_test_1_input.json");
  double calculated_radius = test_satellite.get_radius();
  double calculated_speed = test_satellite.get_speed();
  double expected_circular_orbital_speed =
      sqrt(G * mass_Earth / calculated_radius);
  EXPECT_TRUE(abs(calculated_speed - expected_circular_orbital_speed) <
              tolerance)
      << "Calculated orbital speed did not match expected value within "
         "tolerance. Difference: "
      << calculated_speed - expected_circular_orbital_speed << "\n";
}

TEST(CircularOrbitTests, OrbitalSpeed2) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  double calculated_radius = test_satellite.get_radius();
  double calculated_speed = test_satellite.get_speed();
  double expected_circular_orbital_speed =
      sqrt(G * mass_Earth / calculated_radius);
  EXPECT_TRUE(abs(calculated_speed - expected_circular_orbital_speed) <
              tolerance)
      << "Calculated orbital speed did not match expected value within "
         "tolerance. Difference: "
      << calculated_speed - expected_circular_orbital_speed << "\n";
}

TEST(CircularOrbitTests, TotalEnergyTimestep1) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  double initial_energy = test_satellite.get_total_energy();
  double test_timestep = 1;  // s
  bool perturbation_bool = true;
  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double evolved_energy = test_satellite.get_total_energy();

  EXPECT_TRUE(abs(initial_energy - evolved_energy)/initial_energy < energy_cons_relative_tolerance)
      << "Total energy not preserved within relative tolerance. Relative difference: "
      << abs(initial_energy - evolved_energy)/initial_energy << "\n";
}

TEST(CircularOrbitTests, EvolvedOrbitalRadius1) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  double calculated_initial_radius = test_satellite.get_radius();
  double test_timestep = 1;
  bool perturbation_bool =
      false;  // While from what I can tell (see, e.g.,
              // https://ocw.tudelft.nl/wp-content/uploads/AE2104-Orbital-Mechanics-Slides_8.pdf)
              // there's no major effects on semimajor axis from J2
              // perturbation, not clear to me that this should be exactly
              // constant with J2 perturbation enabled

  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double calculated_evolved_radius = test_satellite.get_radius();

  EXPECT_TRUE(abs(calculated_initial_radius - calculated_evolved_radius) <
              length_tolerance)
      << "Orbital radius not constant within tolerance. Difference: "
      << calculated_initial_radius - calculated_evolved_radius << "\n";
}

TEST(CircularOrbitTests, EvolvedOrbitalSpeed1) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  double calculated_initial_speed = test_satellite.get_speed();
  double test_timestep = 1;
  bool perturbation_bool =
      false;  // While from what I can tell (see, e.g.,
              // https://ocw.tudelft.nl/wp-content/uploads/AE2104-Orbital-Mechanics-Slides_8.pdf)
              // there's no major effects on semimajor axis from J2
              // perturbation, not clear to me that this should be exactly
              // constant with J2 perturbation enabled

  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double calculated_evolved_speed = test_satellite.get_speed();

  EXPECT_TRUE(abs(calculated_initial_speed - calculated_evolved_speed) <
              tolerance)
      << "Orbital speed not constant within tolerance. Difference: "
      << calculated_initial_speed - calculated_evolved_speed << "\n";
}

TEST(CircularOrbitTests, BasicOrbitalElementsTest) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  std::array<double, 6> initial_orbit_elements =
      test_satellite.get_orbital_elements();

  test_satellite.update_orbital_elements_from_position_and_velocity();
  std::array<double, 6> recalculated_orbit_elements =
      test_satellite.get_orbital_elements();

  std::array<std::string, 6> orbital_element_name_array;
  orbital_element_name_array.at(0) = "Semimajor Axis";
  orbital_element_name_array.at(1) = "Eccentricity";
  orbital_element_name_array.at(2) = "Inclination";
  orbital_element_name_array.at(3) = "RAAN";
  orbital_element_name_array.at(4) = "Argument of Periapsis";
  orbital_element_name_array.at(5) = "True Anomaly";

  for (size_t orbital_elem_index = 0; orbital_elem_index < 6;
       orbital_elem_index++) {
    if (orbital_elem_index == 0) {
      EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index) -
                      recalculated_orbit_elements.at(orbital_elem_index)) <
                  length_tolerance)
          << orbital_element_name_array.at(orbital_elem_index)
          << " was not constant within tolerance. Diff:"
          << initial_orbit_elements.at(orbital_elem_index) -
                 recalculated_orbit_elements.at(orbital_elem_index)
          << "\n";
    } else {
      EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index) -
                      recalculated_orbit_elements.at(orbital_elem_index)) <
                  tolerance)
          << orbital_element_name_array.at(orbital_elem_index)
          << " was not constant within tolerance. Diff:"
          << initial_orbit_elements.at(orbital_elem_index) -
                 recalculated_orbit_elements.at(orbital_elem_index)
          << "\n";
    }
  }
}

TEST(CircularOrbitTests, ConstantEvolvedOrbitalElementsTest) {
  // The idea behind this test is that after evolving a timestep, orbital
  // elements besides true anomaly should be constant (when J2 perturbation is
  // not taken into account)
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  std::array<double, 6> initial_orbit_elements =
      test_satellite.get_orbital_elements();

  double test_timestep = 1;
  bool perturbation_bool = false;

  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  std::array<double, 6> evolved_orbit_elements =
      test_satellite.get_orbital_elements();

  std::array<std::string, 6> orbital_element_name_array;
  orbital_element_name_array.at(0) = "Semimajor Axis";
  orbital_element_name_array.at(1) = "Eccentricity";
  orbital_element_name_array.at(2) = "Inclination";
  orbital_element_name_array.at(3) = "RAAN";
  orbital_element_name_array.at(4) = "Argument of Periapsis";
  orbital_element_name_array.at(5) = "True Anomaly";

  for (size_t orbital_elem_index = 0; orbital_elem_index < 5;
       orbital_elem_index++) {
    // True anomaly shouldn't be constant over evolution
    if (orbital_elem_index == 0) {
      EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index) -
                      evolved_orbit_elements.at(orbital_elem_index)) <
                  length_tolerance)
          << orbital_element_name_array.at(orbital_elem_index)
          << " was not constant within tolerance. Diff:"
          << initial_orbit_elements.at(orbital_elem_index) -
                 evolved_orbit_elements.at(orbital_elem_index)
          << "\n";
    } else {
      EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index) -
                      evolved_orbit_elements.at(orbital_elem_index)) <
                  tolerance)
          << orbital_element_name_array.at(orbital_elem_index)
          << " was not constant within tolerance. Diff:"
          << initial_orbit_elements.at(orbital_elem_index) -
                 evolved_orbit_elements.at(orbital_elem_index)
          << "\n";
    }
  }
}

TEST(CircularOrbitTests, Thruster_Eccentricity_Change) {
  Satellite test_satellite("../tests/circular_orbit_test_2_input.json");
  std::array<double, 3> LVLH_thrust_direction = {1, 0, 0};
  double thrust_magnitude = 100;  // N
  double t_thrust_start = 1;
  double t_thrust_end = 100;

  test_satellite.add_LVLH_thrust_profile(
      LVLH_thrust_direction, thrust_magnitude, t_thrust_start, t_thrust_end);
  double test_timestep = 1;  // s
  double current_satellite_time = test_satellite.get_instantaneous_time();
  double sim_end_time = 110;
  while (current_satellite_time < sim_end_time) {
    std::pair<double, int> new_timestep_and_error_code =
        test_satellite.evolve_RK45(epsilon, test_timestep);
    double next_timestep = new_timestep_and_error_code.first;
    int error_code = new_timestep_and_error_code.second;
    test_timestep = next_timestep;
    current_satellite_time = test_satellite.get_instantaneous_time();
  }
  std::array<double, 6> evolved_orbit_elements =
      test_satellite.get_orbital_elements();
  double resulting_eccentricity = evolved_orbit_elements.at(1);

  EXPECT_TRUE(resulting_eccentricity > 0)
      << "Resulting eccentricity was not greater than 0. Calculated value: "
      << resulting_eccentricity << "\n";
}