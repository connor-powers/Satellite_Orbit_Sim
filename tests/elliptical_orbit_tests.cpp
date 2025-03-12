#include <iostream>
#include <gtest/gtest.h>
#include "Satellite.h"
#include "utils.h"

const double non_length_tolerance=pow(10.0,-12);
//Setting a different tolerance for semimajor axis than the other orbital parameters since there appears to be a minimum error associated with converting position and velocity to semimajor axis, 
//best guess is this has to do with the scale of distances being dealt with here (and calculating a_ from position and velocity involves cross product of position and velocity)
const double length_tolerance=pow(10.0,-7);
const double epsilon=pow(10.0,-7);

TEST(EllipticalOrbitTests,EvolvedOrbitalSpeed1){
    //Starting at true anomaly=0 means it's starting at perigee, which is where its orbital speed should be maximum
    Satellite test_satellite("../tests/elliptical_orbit_test_1.json");
    double calculated_initial_speed=test_satellite.get_speed();
    double test_timestep=0.1; //s
    double sim_time=1; //s
    double current_time=test_satellite.get_instantaneous_time();
    double next_timestep=0;
    while (current_time<sim_time){
        next_timestep=test_satellite.evolve_RK45(epsilon,test_timestep);
        test_timestep=next_timestep;
        current_time=test_satellite.get_instantaneous_time();
    }
    double calculated_evolved_speed=test_satellite.get_speed();

    EXPECT_TRUE(calculated_initial_speed>calculated_evolved_speed) << "Perigee speed not larger than calculated evolved speed. Difference: " << calculated_initial_speed-calculated_evolved_speed << "\n";
}

TEST(EllipticalOrbitTests,EvolvedOrbitalSpeed2){
    //Starting at true anomaly=180 means it's starting at apogee, which is where its orbital speed should be minimum
    Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
    double calculated_initial_speed=test_satellite.get_speed();
    double test_timestep=1; //s
    double next_timestep=test_satellite.evolve_RK45(epsilon,test_timestep);
    double calculated_evolved_speed=test_satellite.get_speed();

    EXPECT_TRUE(calculated_initial_speed<calculated_evolved_speed) << "Apogee speed not smaller than calculated evolved speed. Difference: " << calculated_initial_speed-calculated_evolved_speed << "\n";
}


TEST(EllipticalOrbitTests,ConstantEvolvedOrbitalElementsTest){

    Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
    std::array<double,6> initial_orbit_elements=test_satellite.get_orbital_elements();

    double test_timestep=1; //s
    double next_timestep=test_satellite.evolve_RK45(epsilon,test_timestep);
    std::array<double,6> evolved_orbit_elements=test_satellite.get_orbital_elements();

    std::array<std::string,6> orbital_element_name_array;
    orbital_element_name_array.at(0)="Semimajor Axis";
    orbital_element_name_array.at(1)="Eccentricity";
    orbital_element_name_array.at(2)="Inclination";
    orbital_element_name_array.at(3)="RAAN";
    orbital_element_name_array.at(4)="Argument of Periapsis";
    orbital_element_name_array.at(5)="True Anomaly";


    for (size_t orbital_elem_index=0; orbital_elem_index<5;orbital_elem_index++){
        //True anomaly shouldn't be constant over evolution
        if (orbital_elem_index==0){
            EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index)-evolved_orbit_elements.at(orbital_elem_index))<length_tolerance) << orbital_element_name_array.at(orbital_elem_index) << " was not constant within tolerance. Diff:" <<initial_orbit_elements.at(orbital_elem_index)-evolved_orbit_elements.at(orbital_elem_index) << "\n";
        }
        else {
            EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index)-evolved_orbit_elements.at(orbital_elem_index))<non_length_tolerance) << orbital_element_name_array.at(orbital_elem_index) << " was not constant within tolerance. Diff:" <<initial_orbit_elements.at(orbital_elem_index)-evolved_orbit_elements.at(orbital_elem_index) << "\n";
        }
    }
}

TEST(EllipticalOrbitTests,BasicOrbitalElementsTest){

    Satellite test_satellite("../tests/elliptical_orbit_test_2.json");
    std::array<double,6> initial_orbit_elements=test_satellite.get_orbital_elements();

    test_satellite.update_orbital_elements_from_position_and_velocity();
    std::array<double,6> recalculated_orbit_elements=test_satellite.get_orbital_elements();

    std::array<std::string,6> orbital_element_name_array;
    orbital_element_name_array.at(0)="Semimajor Axis";
    orbital_element_name_array.at(1)="Eccentricity";
    orbital_element_name_array.at(2)="Inclination";
    orbital_element_name_array.at(3)="RAAN";
    orbital_element_name_array.at(4)="Argument of Periapsis";
    orbital_element_name_array.at(5)="True Anomaly";



    for (size_t orbital_elem_index=0; orbital_elem_index<6;orbital_elem_index++){
        if (orbital_elem_index==0){
            EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index)-recalculated_orbit_elements.at(orbital_elem_index))<length_tolerance) << orbital_element_name_array.at(orbital_elem_index) << " was not constant within tolerance. Diff:" <<initial_orbit_elements.at(orbital_elem_index)-recalculated_orbit_elements.at(orbital_elem_index) << "\n";
        }
        else {
            EXPECT_TRUE(abs(initial_orbit_elements.at(orbital_elem_index)-recalculated_orbit_elements.at(orbital_elem_index))<non_length_tolerance) << orbital_element_name_array.at(orbital_elem_index) << " was not constant within tolerance. Diff:" <<initial_orbit_elements.at(orbital_elem_index)-recalculated_orbit_elements.at(orbital_elem_index) << "\n";
        }
    }
}