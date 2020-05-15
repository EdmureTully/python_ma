#include "spring_models.h"
#include "haptic_utils.h"

#include <iostream>

namespace spring_models{



basic_spring_model::basic_spring_model()
{
}

basic_spring_model::basic_spring_model(Eigen::Vector3d &null_pos, double spring_constant, double max_force):
    null_position_(null_pos), k_(spring_constant), max_force_(max_force)
{

}

void basic_spring_model::setSpringValues(const Eigen::Vector3d &null_pos,const Eigen::Vector3d& max_limits, double damping_ratio, double max_force, double mass)
{
  null_position_ = null_pos;
  damping_ratio_= damping_ratio;
  max_force_ = max_force;
  mass_ = mass;

  //calculate needed constants
  calculate_spring_constants(max_limits, max_force_);
  calculate_damping_force();
}

Eigen::Vector3d basic_spring_model::calculate_force_feedback(const Eigen::Vector3d pos, double time)
{
    Eigen::Vector3d relativ_pos (pos - null_position_);

    return compute_forces(relativ_pos, time);
}

Eigen::Vector3d basic_spring_model::compute_forces(const Eigen::Vector3d pos, double time)
{
    //calculate velocity compared to last step
    double elapsed_time = time;

    if(elapsed_time < 0.0) elapsed_time = 0.1;

    double x_diff = pos.x() - last_position_.x();
    double y_diff = pos.y() - last_position_.y();
    double z_diff = pos.z() - last_position_.z();

    if(std::abs(x_diff) < 0.0001 ) x_diff = 0.0;
    if(std::abs(y_diff) < 0.0001 ) y_diff = 0.0;
    if(std::abs(z_diff) < 0.0001 ) z_diff = 0.0;

    double velocity_x = 0.25 * (x_diff) / elapsed_time;
    double velocity_y = 0.25 * (y_diff) / elapsed_time;
    double velocity_z = 0.25 * (z_diff) / elapsed_time;

    if(debug_){
      std::cout << "Gesch: x : " << std::to_string(velocity_x) << std::endl;
      std::cout << "Gesch: y : " << std::to_string(velocity_y) << std::endl;
      std::cout << "Gesch: z : " << std::to_string(velocity_z) << std::endl;
    }


    last_position_ = pos;

    //calculate force for each direction: F= -diff(x) * spring constant - damping constant * velocity
    double force_x = - 1.0 * pos.x() * k_x_ - damping_constant_x_ * velocity_x ;
    double force_y = - 1.0 * pos.y() * k_y_ - damping_constant_y_ * velocity_y;
    double force_z = - 1.0 * pos.z() * k_z_ - damping_constant_z_ * velocity_z;

    //if the calculated force is bigger than the maximum force, clamp it accordingly
    if(std::abs(force_x) > max_force_)force_x = haptic_utils::sgn(force_x) * max_force_;
    if(std::abs(force_y) > max_force_)force_y = haptic_utils::sgn(force_y) * max_force_;
    if(std::abs(force_z) > max_force_)force_z = haptic_utils::sgn(force_z) * max_force_;

    if(std::abs(force_x) > max_force_)force_x = 0.0;
    if(std::abs(force_y) > max_force_)force_y = 0.0;
    if(std::abs(force_z) > max_force_)force_z = 0.0;


    if(debug_){
      double deflection = pos.norm();
      std::cout << "Letzte Position: " << std::to_string(last_position_.x()) <<"," <<std::to_string(last_position_.y()) << "," << std::to_string(last_position_.z()) << std::endl;
      std::cout << "aktuelle Position: " << std::to_string(pos.x()) <<"," <<std::to_string(pos.y()) << "," << std::to_string(pos.z()) << std::endl;
      std::cout << "Auslenkung: " << std::to_string(deflection) << std::endl;
      std::cout << "Zeitschritt: " << std::to_string(elapsed_time);
      std::cout << "x Kraft: " << std::to_string(force_vector_[0]) << std::endl;
      std::cout << "y Kraft: " << std::to_string(force_vector_[1]) << std::endl;
      std::cout << "z Kraft " << std::to_string(force_vector_[2]) << std::endl;
      std::cout << "x Geschwindigkeit: " << std::to_string(damping_constant_x_ * velocity_x) << std::endl;
      std::cout << "x Geschwindigkeit: " << std::to_string(damping_constant_y_ * velocity_y) << std::endl;
      std::cout << "y Geschwindigkeit: " << std::to_string(velocity_y) << std::endl;
      std::cout << "z Geschwindigkeit: " << std::to_string(velocity_z) << std::endl;
    }

    return Eigen::Vector3d(force_x, force_y, force_z);
}

void basic_spring_model::calculate_spring_constants(const Eigen::Vector3d &max_limits, double max_force)
{
  // F = diff(x) * spring constant
  k_x_ = max_force / max_limits.x();
  k_y_ = max_force / max_limits.y();
  k_z_ = max_force / max_limits.z();
}

void basic_spring_model::calculate_damping_force()
{
  //damping ratio = damping constant / (2 * sqrt(spring constant * mass))=> solved for damping constant
  damping_constant_x_ = 2.0 * damping_ratio_ + std::sqrt(mass_ * k_x_);
  damping_constant_y_ = 2.0 * damping_ratio_ + std::sqrt(mass_ * k_y_);
  damping_constant_z_ = 2.0 * damping_ratio_ + std::sqrt(mass_ * k_z_);
}

bounding_box_spring_model::bounding_box_spring_model()
{

}

void bounding_box_spring_model::set_up_bounding_box(Eigen::Vector3d center, double height, double width, double length)
{
    b_box_ = geometry::bounding_box(center, height, width, length);
}

}


