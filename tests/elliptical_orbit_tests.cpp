#include <gtest/gtest.h>

#include <iostream>

#include "Satellite.h"
#include "utils.h"

const double tolerance = pow(10.0, -12);
// Setting a different tolerance for semimajor axis and orbital radius than the other orbital
// parameters since there appears to be a minimum error associated with
// converting position and velocity to semimajor axis, best guess is this has to
// do with the scale of distances and/or velocities being dealt with here
const double length_tolerance = pow(10.0, -7);
const double epsilon = pow(10.0, -11);
const double energy_cons_relative_tolerance = pow(10.0, -5);

TEST(EllipticalOrbitTests, EvolvedOrbitalSpeed1) {
  // Starting at true anomaly=0 means it's starting at perigee, which is where
  // its orbital speed should be maximum
  Satellite test_satellite("../tests/elliptical_orbit_test_1.json");
  double calculated_initial_speed = test_satellite.get_speed();
  double test_timestep = 0.1;  // s
  double sim_time = 1;         // s
  double current_time = test_satellite.get_instantaneous_time();
  double next_timestep = 0;
  while (current_time < sim_time) {
    std::pair<double, int> new_timestep_and_error_code =
        test_satellite.evolve_RK45(epsilon, test_timestep);
    next_timestep = new_timestep_and_error_code.first;
    int error_code = new_timestep_and_error_code.second;
    test_timestep = next_timestep;
    current_time = test_satellite.get_instantaneous_time();
  }
  double calculated_evolved_speed = test_satellite.get_speed();

  EXPECT_TRUE(calculated_initial_speed > calculated_evolved_speed)
      << "Perigee speed not larger than calculated evolved speed. Difference: "
      << calculated_initial_speed - calculated_evolved_speed << "\n";
}

TEST(EllipticalOrbitTests, EvolvedOrbitalSpeed2) {
  // Starting at true anomaly=180 means it's starting at apogee, which is where
  // its orbital speed should be minimum
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double calculated_initial_speed = test_satellite.get_speed();
  double test_timestep = 1;  // s
  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double calculated_evolved_speed = test_satellite.get_speed();

  EXPECT_TRUE(calculated_initial_speed < calculated_evolved_speed)
      << "Apogee speed not smaller than calculated evolved speed. Difference: "
      << calculated_initial_speed - calculated_evolved_speed << "\n";
}

TEST(EllipticalOrbitTests, ConstantEvolvedOrbitalElementsTest) {
  // The idea behind this test is that after evolving a timestep, orbital
  // elements besides true anomaly should be constant (when J2 perturbation is
  // not taken into account)

  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  std::array<double, 6> initial_orbit_elements =
      test_satellite.get_orbital_elements();

  double test_timestep = 1;  // s
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

TEST(EllipticalOrbitTests, BasicOrbitalElementsTest) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
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

TEST(EllipticalOrbitTests,OrbitalRadiusCalcs1) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double orbital_radius_perifocal=test_satellite.get_radius();
  double orbital_radius_ECI=test_satellite.get_radius_ECI();
  EXPECT_TRUE(abs(orbital_radius_perifocal - orbital_radius_ECI) < length_tolerance)
      << "Difference between orbital radii calculated with "
      " perifocal and ECI coordinates: "
      << orbital_radius_perifocal - orbital_radius_ECI <<"\n";
}

TEST(EllipticalOrbitTests,OrbitalRadiusCalcs2) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double test_timestep = 0.1;  // s
  bool perturbation_bool = false;
  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double orbital_radius_perifocal=test_satellite.get_radius();
  double orbital_radius_ECI=test_satellite.get_radius_ECI();

  EXPECT_TRUE(abs(orbital_radius_perifocal - orbital_radius_ECI) < length_tolerance)
      << "Difference between evolved orbital radii calculated with "
      " perifocal and ECI coordinates: "
      << orbital_radius_perifocal - orbital_radius_ECI <<"\n";
}

TEST(EllipticalOrbitTests,OrbitalSpeedCalcs1) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double orbital_speed_perifocal=test_satellite.get_speed();
  double orbital_speed_ECI=test_satellite.get_speed_ECI();
  EXPECT_TRUE(abs(orbital_speed_ECI - orbital_speed_perifocal) < tolerance)
      << "Difference between orbital speeds calculated with "
      " perifocal and ECI coordinates: "
      << orbital_speed_ECI - orbital_speed_perifocal <<"\n";
}

TEST(EllipticalOrbitTests,OrbitalSpeedCalcs2) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double test_timestep = 0.1;  // s
  bool perturbation_bool = false;
  std::pair<double, int> new_timestep_and_error_code =
      test_satellite.evolve_RK45(epsilon, test_timestep, perturbation_bool);
  double next_timestep = new_timestep_and_error_code.first;
  int error_code = new_timestep_and_error_code.second;
  double orbital_speed_perifocal=test_satellite.get_speed();
  double orbital_speed_ECI=test_satellite.get_speed_ECI();

  EXPECT_TRUE(abs(orbital_speed_ECI - orbital_speed_perifocal) < tolerance)
      << "Difference between evolved orbital speeds calculated with "
      " perifocal and ECI coordinates: "
      << orbital_speed_ECI - orbital_speed_perifocal <<"\n";
}

TEST(EllipticalOrbitTests, TotalEnergyTimestep1) {
  Satellite test_satellite("../tests/elliptical_orbit_test_1.json");
  double initial_energy = test_satellite.get_total_energy();
  double test_timestep = 0.1;  // s
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

TEST(EllipticalOrbitTests, TotalEnergyTimestep2) {
  Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
  double initial_energy = test_satellite.get_total_energy();
  double test_timestep = 0.1;  // s
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

TEST(EllipticalOrbitTests, TotalEnergyTimestep3) {
  Satellite test_satellite("../tests/elliptical_orbit_test_3.json");
  double initial_energy = test_satellite.get_total_energy();
  double test_timestep = 0.1;  // s
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

TEST(EllipticalOrbitTests, DragTest1) {
  Satellite test_satellite_withdrag("../tests/elliptical_orbit_test_4.json");
  Satellite test_satellite_nodrag("../tests/elliptical_orbit_test_4.json");
  // Drag parameters
  double F_10 = 100;  // Solar radio ten centimeter flux
  double A_p = 120;   // Geomagnetic A_p index
  // Collect drag parameters into a pair with F_10 first and A_p second
  std::pair<double, double> drag_elements = {F_10, A_p};
  double test_timestep = 0.1;  // s
  bool perturbation_bool = true;
  double total_sim_time = 10; // s
  double current_time = test_satellite_nodrag.get_instantaneous_time();
  while (current_time < total_sim_time) {
  std::pair<double, int> new_timestep_and_error_code =
      test_satellite_nodrag.evolve_RK45(epsilon, test_timestep, perturbation_bool,
        false);
      double next_timestep = new_timestep_and_error_code.first;
      test_timestep = next_timestep;
      int error_code = new_timestep_and_error_code.second;
      current_time = test_satellite_nodrag.get_instantaneous_time();
  }
  double no_drag_semimajor_axis = test_satellite_nodrag.get_orbital_element("Semimajor Axis");


  current_time = test_satellite_withdrag.get_instantaneous_time();
  while (current_time < total_sim_time) {
  std::pair<double, int> new_timestep_and_error_code =
  test_satellite_withdrag.evolve_RK45(epsilon, test_timestep, perturbation_bool,
        false);
      double next_timestep = new_timestep_and_error_code.first;
      test_timestep = next_timestep;
      int error_code = new_timestep_and_error_code.second;
      current_time = test_satellite_withdrag.get_instantaneous_time();
  }
  double with_drag_semimajor_axis = test_satellite_withdrag.get_orbital_element("Semimajor Axis");

  EXPECT_TRUE(no_drag_semimajor_axis > with_drag_semimajor_axis)
      << "Semimajor axis after evolution wasn't lower when drag was introduced. "
      "This isn't expected behavior.\n";
}
