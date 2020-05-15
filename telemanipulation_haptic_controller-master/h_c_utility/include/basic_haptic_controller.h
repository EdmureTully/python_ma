#ifndef BASIC_HAPTIC_CONTROLLER_H
#define BASIC_HAPTIC_CONTROLLER_H

#include "control_toolbox/pid.h"
#include "ros/duration.h"
#include <eigen3/Eigen/Dense>


namespace haptic_controller {

class BasicHapticController{

public:
  BasicHapticController();

  /**
  * @brief sets values of pid controller
  * @param p proportional (spring constant)
  * @param i integrative (mass?)
  * @param d derivative (damping constant)
  * @param i_max maximum integral windup(boundary for integral values)
  * @param i_min minimum integral windup(boundary for integral values)
  */
  void  setPIDValues(double p, double i, double d, double i_max, double i_min);

  /**
  * @brief update PID loop one time
  * @param current_pos current position
  * @param desired_pos goal position
  * @param dur time step
  */
  Eigen::Vector3d calculatePIDStepForce(Eigen::Vector3d& current_pos, Eigen::Vector3d& desired_pos, ros::duration& dur);

private:

  control_toolbox::PID pid_control;

};


}





#endif // BASIC_HAPTIC_CONTROLLER_H
