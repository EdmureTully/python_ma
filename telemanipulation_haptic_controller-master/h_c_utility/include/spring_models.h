#ifndef SPRING_MODELS_H
#define SPRING_MODELS_H

#include "bounding_geometry.h"

#include<eigen3/Eigen/Dense>

namespace spring_models {


class basic_spring_model
{
public:

    /**
    * @brief standard constructor
    */
    basic_spring_model();

    /**
    * @brief constructor
    * @param null_pos starting position of the controller
    * @param spring_constant for Hooke's law
    * @param max_force maximum force to be exerted
    */
    basic_spring_model(Eigen::Vector3d& null_pos, double spring_constant, double max_force);

    /**
    * @brief sets members of the spring model
    * @param null_pos starting positon equals desired position(x,y,z)-coordinates
    * @param damping_ratio damping ratio, needed to calculate the damping constant for each dimension
    * @param max_force maximum force to be applied
    * @param mass particle's mass
    */
    void setSpringValues(const Eigen::Vector3d& null_pos,const Eigen::Vector3d& max_limits, double damping_ratio = 12.5, double max_force = 12.0, double mass = 1.0);

    /**
    * @brief calculates the force feedback in x,y,z-direction
    * @return forces to be exerted in x,y,z direction
    */
    Eigen::Vector3d calculate_force_feedback(const Eigen::Vector3d pos, double time = 100.0);

private:

    /**
    * @brief computes the forces exerted by the spring and saves it in force_vector_
    * @param pos current position of the particle
    * @param time time step
    */
    Eigen::Vector3d compute_forces(const Eigen::Vector3d pos, double time);

    /**
    * @brief calculates the spring constants for each dimension (x, y, z)
    * @param max_limits limits in each direction
    * @param max_force maximum force that can be exerted by the spring
    */
    void calculate_spring_constants(const Eigen::Vector3d& max_limits, double max_force);

    /**
    * @brief  calculates the damping force for each dimension(x,y,z); needs damping ratio, mass and the spring constant for each dimension
    */
    void calculate_damping_force();

    Eigen::Vector3d null_position_;/**< initial starting position of the controller*/

    Eigen::Vector3d force_vector_;/**< force vector: x, y, z component*/

    Eigen::Vector3d last_vel = Eigen::Vector3d(0.0, 0.0, 0.0);

    double k_;/**< spring constant if there is just one spring for alle dimensions*/

    double k_x_ = 240.0;/**< spring constant x dimension spring*/

    double k_y_ = 120.0;/**< spring constant y dimension spring*/

    double k_z_ = 134.0;/**< spring constant y dimension spring*/

    double damping_constant_x_;/**< spring damping constant x dimension spring*/

    double damping_constant_y_;/**< spring damping constant y dimension spring*/

    double damping_constant_z_;/**< spring damping constant z dimension spring*/

    double damping_ratio_ = 12.5;/**< damping ration if there is just one spring for all dimensions */

    double max_force_ = 12.0;/**< maximum force that can be exerted by the spring */

    double mass_ = 1.0;/**< mass of the particle attached to the spring */

    Eigen::Vector3d last_position_;/**< last position of the particle attached to the spring */

    Eigen::Vector3d last_force_ = Eigen::Vector3d(0.0,0.0,0.0);/**< last force exerted by the spring */

    bool debug_ = false;/**< if true, prints to console */

};

class bounding_box_spring_model
{

public:

    bounding_box_spring_model();

    void set_up_bounding_box(Eigen::Vector3d center, double height, double width, double length);


private:


   geometry::bounding_box b_box_;
};


}








#endif // SPRING_MODELS_H
