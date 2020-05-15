#include "basic_haptic_controller.h"
#include "haptic_wrapper.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dhdc.h"

#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>


fd_haptic_hw_test::FdHapticWrapper init_device(){

    fd_haptic_hw_test::FdDeviceType model(fd_haptic_hw_test::FdDeviceModel::OMEGA_7, fd_haptic_hw_test::FdDeviceSide::RIGHT);
    fd_haptic_hw_test::FdHapticWrapper fd_wraper(model);
    return fd_wraper;
}


namespace haptic_controller {

BasicHapticController::BasicHapticController(){

}

BasicHapticController::setPIDValues(double p, double i, double d, double i_max, double i_min){
  pid_control.initPid(p, i, d, i_max, i_min);
}

Eigen::Vector3d BasicHapticController::calculatePIDStepForce(Eigen::Vector3d& current_pos, Eigen::Vector3d& desired_pos, ros::duration& dur){

  Eigen::Vector3d diff(desired_pos - current_pos);

  double x_force, y_force, z_force;

  x_force = pid_control.updatePid(diff.x(), dur);
  y_force = pid_control.updatePid(diff.y(), dur);
  z_force = pid_control.updatePid(diff.z(), dur);

  Eigen::Vector3d force_vector(x_force, y_force, z_force);
  return force_vector;
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_haptic_controller");

  ros::NodeHandle n;





  fd_haptic_hw::FdHapticWrapper dev = init_device();

  Eigen::Vector3d center(0.0, 0.0, 0.0);

  haptic_controller::BasicHapticController basic_controller;
    //werte nach der docu
  basic_controller.setPIDValues(6.0, 1.0, 2.0, 0.3, -0.3);

  fd_haptic_hw::PositionArray pos (std::array<double, 3>{0.0, 0.0, 0.0});


  Eigen::Vector3d desired_position, current_position(*pos.positionX(),*pos.positionY(),*pos.positionZ());


  ros::Time last_time = ros::Time::now();

  int done(0);

  while(!done){

    ros::Time time = ros::Time::now();
    pos = dev.getPosition();
    current_position = Eigen::Vector3d(*pos.positionX(), *pos.positionY() ,*pos.positionZ());


    Eigen::Vector3d forces = basic_controller.calculatePIDStepForce(current_position, center, (last_time - time));
    last_time = time;

    if(!dev.setCartesianForce(forces)){
      std::cout << "Forces could not be set" << std::endl;
    }

  }


}





/*
void keyboardCallback(const std_msgs::Char::ConstPtr& msg){
  int c = msg->data;
  std::cout << "in callback: " << std::to_string(c) << std::endl;


  if(c == 'd'){
    std::cout << "D-Wert um 10 erhoeht" << std::endl;
    d_val += 1.0 * factor;
    pid_val_changed = true;
  }else if ( c == 'D') {
    std::cout << "D-Wert um 5 erhoeht" << std::endl;
    d_val += 0.50 *  factor;
    pid_val_changed = true;
  }else if ( c == 's') {
    std::cout << "D-Wert um 10 verringert" << std::endl;
    d_val -= 1.0 * factor;
    pid_val_changed = true;
  }else if ( c == 'S') {
    std::cout << "D-Wert um 5 verringert" << std::endl;
    d_val -= 0.50 * factor;
    pid_val_changed = true;
  }else if ( c == 'p') {
    std::cout << "P-Wert um 10 erhoeht" << std::endl;
    p_val += 10.0;
    pid_val_changed = true;
  }else if ( c == 'P') {
    std::cout << "P-Wert um 5 erhoeht" << std::endl;
    p_val += 5.0;
    pid_val_changed = true;
  }else if ( c == 'o') {
    std::cout << "P-Wert um 10 verringert" << std::endl;
    p_val -= 10.0;
    pid_val_changed = true;
  }else if ( c == 'O') {
    std::cout << "P-Wert um 5 verringert" << std::endl;
    p_val -= 5.0;
    pid_val_changed = true;
  }else if ( c == 'i') {
    std::cout << "I-Wert um 10 erhoeht" << std::endl;
    i_val += 10.0;
    pid_val_changed = true;
  }else if ( c == 'I') {
    std::cout << "I-Wert um 5 erhoeht" << std::endl;
    i_val += 5.0;
    pid_val_changed = true;
  }else if ( c == 'u') {
    std::cout << "I-Wert um 10 verringert" << std::endl;
    i_val -= 10.0;
    pid_val_changed = true;
  }else if ( c == 'U') {
    std::cout << "I-Wert um 5 verringert" << std::endl;
    i_val -= 5.0;
    pid_val_changed = true;
  }else if(c == 'f'){
    std::cout << "Faktor mit 0.1 multipliziert"  << std::endl;
    factor *= 0.1;F
    pid_val_changed = true;
  }else if(c == 'F'){
    std::cout << "Faktor mit 10 multipliziert"  << std::endl;
    factor *= 10.0;
    pid_val_changed = true;
  }else if (c == 'x') {
    d_val *= factor;
    pid_val_changed = true;
  }else if (c == 'X') {
    d_val /= factor;
    pid_val_changed = true;
  }
  if(pid_val_changed){
    pid_val_changed = false;
    basic_controller.setPIDValues(p_val, i_val, d_val, i_min, i_max);
    std::string pid_string = std::string("PID-Werte aktulisiert: ") + std::string("p: ") + std::to_string(p_val) + std::string(" i: ") + std::to_string(i_val) + std::string("d: ") + std::to_string(d_val) + std::string(" Faktor") + std::to_string(factor);
    data_writer::get_instance().write_string_to_file(pid_string);
  }


}
*/
