#include <iostream>
#include <cmath>
#include "Satellite.h"
//baselining cartesian coordinates in ECI frame

std::array<double,3> calculate_orbital_acceleration(const std::array<double,3> input_r_vec)
{
    //orbital acceleration = -G m_Earth/distance^3 * r_vec (just based on rearranging F=ma with a the acceleration due to gravitational attraction between satellite and Earth https://en.wikipedia.org/wiki/Newton%27s_law_of_universal_gravitation)
    //going to be assuming Earth's position doesn't change for now
    //also assuming Earth is spherical, can loosen this assumption in the future
    std::array<double,3> acceleration_vec=input_r_vec; //shouldn't need to explicitly call copy function because input_r_vec is passed by value not ref
    


    const double distance=sqrt(input_r_vec.at(0)*input_r_vec.at(0) + input_r_vec.at(1)*input_r_vec.at(1) + input_r_vec.at(2)*input_r_vec.at(2));
    const double overall_factor= -G*mass_Earth/pow(distance,3);



    for (size_t ind=0;ind<input_r_vec.size();ind++){
        acceleration_vec.at(ind)*=overall_factor;
    }

    return acceleration_vec;
}


std::pair<std::array<double,3>,std::array<double,3>> propagate_orbit_RK4(std::function<std::array<double,3>(const std::array<double,3> input_r_vec)> input_acceleration_calculation_function, const double input_initial_t,const std::array<double,3> input_initial_position_vec,const std::array<double,3> input_initial_velocity_vec,const double input_step_size)
{
    //https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method
    //https://en.wikipedia.org/wiki/Euler_method
    std::pair<std::array<double,3>,std::array<double,3>> position_and_vel_at_next_timestep={}; //format: {\vec{x}(t+\Delta t),\vec{v}(t+\Delta t)}

    double t_1=input_initial_t;
    double t_2=input_initial_t+input_step_size/2;
    double t_3=t_2;
    double t_4=input_initial_t+input_step_size;


    std::array<double,6> y_n={input_initial_position_vec.at(0),input_initial_position_vec.at(1),input_initial_position_vec.at(2),input_initial_velocity_vec.at(0),input_initial_velocity_vec.at(1),input_initial_velocity_vec.at(2)};


    //k=1
    //velocity is the f() for calculating position via RK4, acceleration is the f() for calculating velocity via RK4
    std::array<double,6> k_1={0,0,0,0,0,0};
    for (size_t ind=0;ind<3;ind++){
        k_1.at(ind)=y_n.at(ind+3);
    }

    std::array<double,3> a_n=calculate_orbital_acceleration(input_initial_position_vec);

    for (size_t ind=3;ind<y_n.size();ind++){
        k_1.at(ind)=a_n.at(ind-3);
    }

    //now using Euler method to approximate the next k step values
    //k=2





    return position_and_vel_at_next_timestep;
}