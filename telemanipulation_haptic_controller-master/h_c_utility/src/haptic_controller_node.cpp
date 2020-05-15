#include "haptic_controller/haptic_controller_node.h"
#include "haptic_wrapper.h"
#include "haptic_utils.h"


//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Time.h"

//other


//0006755 p:690.0 d:0.001121
//03.02
//d:0.000089323 d: 0.0092125 p 2.5

fd_haptic_hw_test::FdHapticWrapper init_device(){

    fd_haptic_hw_test::FdDeviceType model(fd_haptic_hw_test::FdDeviceModel::OMEGA_7, fd_haptic_hw_test::FdDeviceSide::RIGHT);
    fd_haptic_hw_test::FdHapticWrapper fd_wraper(model);
    return fd_wraper;
}

namespace haptic_controller {

BasicHapticController::BasicHapticController(){

}

BasicHapticController::BasicHapticController(Eigen::Vector3d &pos_limits, Eigen::Vector3d& f_limits):
  position_limits(pos_limits), force_limits(f_limits)
{

}

void BasicHapticController::setPIDValues(double p, double i, double d, double i_max, double i_min){

  p_val_pos_based = p;
  i_val_pos_based = i;
  d_val_pos_based = d;
  i_min_pos_based = i_min;
  i_max_pos_based = i_max;

  pid_control_position.initPid(p_val_pos_based, i_val_pos_based, d_val_pos_based, i_min_pos_based, i_max_pos_based);
}

void BasicHapticController::setPIDValuesForceX(double p, double i, double d, double i_max, double i_min)
{

  p_val_force_x = p;
  i_val_force_x = i;
  d_val_force_x = d;
  i_min_force_x = i_min;
  i_max_force_x = i_max;
  pid_control_force_x.initPid(p_val_force_x, i_val_force_x, d_val_force_x, i_max_force_x, i_min_force_x);
}

void BasicHapticController::setPIDValuesForceY(double p, double i, double d, double i_max, double i_min)
{

  p_val_force_y = p;
  i_val_force_y = i;
  d_val_force_y = d;
  i_min_force_y = i_min;
  i_max_force_y = i_max;
  pid_control_force_y.initPid(p_val_force_y, i_val_force_y, d_val_force_y, i_max_force_y, i_min_force_y);
}

void BasicHapticController::setPIDValuesForceZ(double p, double i, double d, double i_max, double i_min)
{
  p_val_force_z = p;
  i_val_force_z = i;
  d_val_force_z = d;
  i_min_force_z = i_min;
  i_max_force_z = i_max;
  pid_control_force_z.initPid(p_val_force_z, i_val_force_z, d_val_force_z, i_max_force_z, i_min_force_z);
}

void BasicHapticController::setPIDValuesForce()
{
  setPIDValuesForceX(p_val_force_x, i_val_force_x, d_val_force_x, i_max_force_x, i_min_force_x);
  setPIDValuesForceY(p_val_force_y, i_val_force_y, d_val_force_y, i_max_force_y, i_min_force_y);
  setPIDValuesForceZ(p_val_force_z, i_val_force_z, d_val_force_z, i_max_force_z, i_min_force_z);
}

Eigen::Vector3d BasicHapticController::calculatePIDStepPosition(Eigen::Vector3d& current_pos, ros::Duration& dur){

  Eigen::Vector3d diff(desired_position - current_pos);

  double x_step, y_step(0.00), z_step(0.00);

  x_step = pid_control_position.computeCommand(diff.x(), dur);
  y_step = pid_control_position.computeCommand(diff.y(), dur);
  z_step = pid_control_position.computeCommand(diff.z(), dur);

  if(std::abs(x_step) > position_limits.x()){
    x_step = haptic_utils::sgn(x_step) * position_limits.x();
   }
  if(std::abs(y_step) > position_limits.y()){
    y_step = haptic_utils::sgn(y_step) * position_limits.y();
  }
  if(std::abs(z_step) > position_limits.z()){
    z_step = haptic_utils::sgn(z_step) * position_limits.z();
  }

  if(debug){

    std::cout << "X-Position momentan: " << std::to_string(current_pos.x()) << std::endl;
    std::cout << "Y-Position momentan: " << std::to_string(current_pos.y()) << std::endl;
    std::cout << "Z-Position momentan: " << std::to_string(current_pos.z()) << std::endl;

    std::cout << "X-Position Ziel: " << std::to_string(desired_position.x()) << std::endl;
    std::cout << "Y-Position Ziel: " << std::to_string(desired_position.y()) << std::endl;
    std::cout << "Z-Position Ziel: " << std::to_string(desired_position.z()) << std::endl;

    std::cout << "Schritt x: " << std::to_string(x_step) << std::endl;
    std::cout << "Schritt y: " << std::to_string(y_step) << std::endl;
    std::cout << "Schritt z: " << std::to_string(z_step) << std::endl;
    std::cout << "Zeitschritt: " << std::to_string(dur.toSec()) << std::endl;

  }

  Eigen::Vector3d step_vector(x_step, y_step, z_step);

  return step_vector;
}


Eigen::Vector3d BasicHapticController::calculatePIDStepForceThreeDim(Eigen::Vector3d &current_force, ros::Duration &dur)
{
  Eigen::Vector3d diff(current_force - desired_force);



  double x_step(0.0), y_step(0.00), z_step(0.00);

  x_step = pid_control_force_x.computeCommand(diff.x(), dur);
  y_step = pid_control_force_y.computeCommand(diff.y(), dur);
  z_step = pid_control_force_z.computeCommand(diff.z(), dur);

  if(std::abs(x_step) > force_limits.x()){
    x_step = haptic_utils::sgn(x_step) * force_limits.x();
   }
  if(std::abs(y_step) > force_limits.y()){
    y_step = haptic_utils::sgn(y_step) *  force_limits.y();
  }
  if(std::abs(z_step) > force_limits.z()){
    z_step = haptic_utils::sgn(z_step) * force_limits.z();
  }

  if(debug){

    std::cout << "X-Position momentan: " << std::to_string(current_force.x()) << std::endl;
    std::cout << "Y-Position momentan: " << std::to_string(current_force.y()) << std::endl;
    std::cout << "Z-Position momentan: " << std::to_string(current_force.z()) << std::endl;
    std::cout << "X-Position Ziel: " << std::to_string(desired_force.x()) << std::endl;
    std::cout << "Y-Position Ziel: " << std::to_string(desired_force.y()) << std::endl;
    std::cout << "Z-Position Ziel: " << std::to_string(desired_force.z()) << std::endl;
    std::cout << "In calculatePIDStepForce: " << std::endl;
    std::cout << "Kraft x nach PID: " << std::to_string(x_step) << std::endl;
    std::cout << "Kraft y nach PID: " << std::to_string(y_step) << std::endl;
    std::cout << "Kraft z nach PID: " << std::to_string(z_step) << std::endl;
    std::cout << "Kraft x Soll " << std::to_string(diff.x()) << std::endl;
    std::cout << "Kraft y Soll: " << std::to_string(diff.y()) << std::endl;
    std::cout << "Kraft z Soll: " << std::to_string(diff.z()) << std::endl;
    std::cout << "Zeitschritt: " << std::to_string(dur.toSec()) << std::endl;
  }


  Eigen::Vector3d step_vector(x_step, y_step, z_step);

  return step_vector;
}

void BasicHapticController::setSpringValues(Eigen::Vector3d &null_pos, Eigen::Vector3d& max_limits, double damping_ratio, double max_force, double mass)
{
  spring.setSpringValues(null_pos, max_limits, damping_ratio, max_force, mass);
}

void BasicHapticController::setDesiredPosition(Eigen::Vector3d &des_pos)
{
  desired_position = des_pos;
}

void BasicHapticController::setDesiredForce(Eigen::Vector3d &des_force)
{
  desired_force = des_force;
}

void BasicHapticController::setPositionLimits(Eigen::Vector3d &pos_limits)
{
  position_limits = pos_limits;
}

void BasicHapticController::setForceLimits(Eigen::Vector3d &f_limits)
{
  force_limits = f_limits;
}

Eigen::Vector3d BasicHapticController::positionBasedStep(Eigen::Vector3d &current_pos, ros::Duration &time_step)
{
  Eigen::Vector3d step = calculatePIDStepPosition(current_pos, time_step);
  Eigen::Vector3d ret_vec = current_pos + step;
  return ret_vec;
}

Eigen::Vector3d BasicHapticController::forceBasedStep(Eigen::Vector3d &current_pos, ros::Duration& time_step)
{
  Eigen::Vector3d force_vec = spring.calculate_force_feedback(current_pos, time_step.toSec());

  Eigen::Vector3d force_step = calculatePIDStepForceThreeDim(force_vec, time_step);
  force_step = last_force + (force_step - last_force);
  last_force = force_step;

  if(debug){

    std::cout << "x Kraft Feder: " << std::to_string(force_vec.x()) << std::endl;
    std::cout << "y Kraft Feder: " << std::to_string(force_vec.y()) << std::endl;
    std::cout << "z Kraft Feder: " << std::to_string(force_vec.z()) << std::endl;
    std::cout << "x Kraft PID: " << std::to_string(force_step.x()) << std::endl;
    std::cout << "y Kraft PID: " << std::to_string(force_step.y()) << std::endl;
    std::cout << "z Kraft PID: " << std::to_string(force_step.z()) << std::endl;
  }

  return force_step;
}

PositionBasedController::PositionBasedController()
{
  last_time = ros::Time::now();  
}

PositionBasedController::PositionBasedController(Eigen::Vector3d &center_point):
  center(center_point)
{
  last_time = ros::Time::now();
}


void PositionBasedController::initBoundingCone(Eigen::Vector3d &tip, double height, double opening_angle, double radius, unsigned int base_points)
{
  bounding_cone.init_cone(tip, height, opening_angle, radius, base_points);

  for(const Eigen::Vector3d& pt : bounding_cone.get_base_points()){
    haptic_utils::write_eigen_vector3_to_console(pt);
  }

}

void PositionBasedController::initBoundingBox(Eigen::Vector3d &center, double height, double width, double length)
{
  bounding_box.update_bounding_box(center, height, width, length);
  std::cout << "Box points of bounding box: " << std::endl;
  for(const Eigen::Vector3d& pt : bounding_box.get_box_points()){
    haptic_utils::write_eigen_vector3_to_console(pt);
  }
}

bool PositionBasedController::checkPositionInsideBoundingObject(Eigen::Vector3d& new_pos)
{
  bool inside = false;

  if(bounding_box_selected){
    inside = geo_procs.point_inside_object(bounding_box, new_pos);
  }
  if(bounding_cone_selected){
    inside = geo_procs.point_inside_object(bounding_cone, new_pos);
  }
  return inside;
}

bool PositionBasedController::updateHoldPosition(Eigen::Vector3d &new_pos, ros::Time &time)
{


  bool time_passed = updateTime(time), new_hold_position(false);

  if((last_set_position - new_pos).norm() < min_distance){
    return false;
  }

  if(time_passed){
    new_hold_position = updatePosition(new_pos);

    if(new_hold_position){
      last_set_position = last_position;
    }
  }

  return new_hold_position;
}

void PositionBasedController::setSpringValues(Eigen::Vector3d &null_pos, Eigen::Vector3d &max_limits, double damping_ratio, double max_force, double mass)
{
    spring.setSpringValues(null_pos, max_limits, damping_ratio, max_force, mass);
}

bool PositionBasedController::updatePosition(Eigen::Vector3d &new_pos)
{
  bool position_changed(true);

  Eigen::Vector3d diff(last_position - new_pos);

  double magnitude = diff.norm();
  if(debug)

  if(magnitude > highest_mag)highest_mag = magnitude;

  if(avg_mag < 0.0){
    avg_mag = magnitude * 1000;
  }else{
    avg_mag = (avg_mag + magnitude * 1000) * 0.5;
  }
  //check if significant change in position
  if(magnitude > min_distance){
    position_changed = true;
    last_position = new_pos;
  }
  //if not, update last position in a k-means kind of way
  else{
    position_changed = false;
    last_position = (new_pos + last_position) * 0.5;
  }
  if(debug){
    std::cout << "Magnitude between latest position and new position: " << std::to_string(magnitude) << std::endl;
  }

  return position_changed;
}

bool PositionBasedController::updateTime(ros::Time &time)
{
  std::cout << "Time difference in updateTime: " << std::to_string((time - last_time).toSec()) << std::endl;

  if((time - last_time).toSec() > time_limit){
    last_time = time;
    return true;
  }
  return false;
}

Eigen::Vector3d PositionBasedController::get_last_position() const
{
  return last_position;
}

double PositionBasedController::get_highest_magnitude() const
{
  return highest_mag;
}

double PositionBasedController::get_avg_magnitude() const
{
  return avg_mag / 1000.0;
}



Eigen::Vector3d PositionBasedController::calculateSpringForce(Eigen::Vector3d &pos)
{
  return spring.calculate_force_feedback(pos);
}

void PositionBasedController::switchBoundingModell()
{
  bounding_box_selected = !bounding_box_selected;
  bounding_cone_selected = !bounding_cone_selected;
}

BasicHapticController initVelocityBasedController(Eigen::Vector3d center, haptic_utils::omega_device_limits& om_limits, double max_force){

  Eigen::Vector3d pos_lim = om_limits.get_limits();
  Eigen::Vector3d force_lim(max_force, max_force , max_force);
  haptic_controller::BasicHapticController basic_controller(pos_lim, force_lim);
  basic_controller.setPIDValuesForce();
  //set spring values, center position of spring is (0,0,0)
  basic_controller.setSpringValues(center, pos_lim);
  return basic_controller;
}

}




fd_haptic_hw_test::JointArray transformEigenToJointArray(Eigen::Vector3d force){
  fd_haptic_hw_test::JointArray joint_array;
  *joint_array.jointX() = force.x();
  *joint_array.jointY() = force.y();
  *joint_array.jointZ() = force.z();
  return joint_array;
}

fd_haptic_hw_test::PositionArray transformEigenToPositionArray(Eigen::Vector3d position){
  fd_haptic_hw_test::PositionArray pos_array;
  *pos_array.positionX() = position.x();
  *pos_array.positionY() = position.y();
  *pos_array.positionZ() = position.z();
  return pos_array;
}

void usePositionBasedController(fd_haptic_hw_test::FdHapticWrapper& dev, haptic_controller::PositionBasedController& pos_controller){

  fd_haptic_hw_test::PositionArray pos (std::array<double, 3>{0.0, 0.0, 0.0});
  pos = dev.getPosition();
  Eigen::Vector3d current_position =  Eigen::Vector3d(*pos.positionX(), *pos.positionY() ,*pos.positionZ());
  Eigen::Vector3d center;
  haptic_utils::omega_device_limits om_limits;
  center = om_limits.get_center_device();

  current_position -= center;

  ros::Time time = ros::Time::now();

  bool inside = pos_controller.checkPositionInsideBoundingObject(current_position);

  if(!inside){
    fd_haptic_hw_test::JointArray joint_array = transformEigenToJointArray(pos_controller.calculateSpringForce(current_position));
    dev.write(joint_array);
  }

  if(pos_controller.updateHoldPosition(current_position, time) && inside){
    Eigen::Vector3d latest_pos = pos_controller.get_last_position();
    pos = transformEigenToPositionArray(latest_pos);
    dev.moveToPosition(pos);
  }
}

void useVelocityBasedController(fd_haptic_hw_test::FdHapticWrapper& dev, haptic_controller::BasicHapticController& vel_controller, ros::Time& last_time)
{


      ros::Time time = ros::Time::now();

      ros::Duration dur = time - last_time;
      Eigen::Vector3d center;
      haptic_utils::omega_device_limits om_limits;
      center = om_limits.get_center_device();

      // get current positon
      fd_haptic_hw_test::PositionArray pos = dev.getPosition();
      Eigen::Vector3d current_position =  Eigen::Vector3d(*pos.positionX(), *pos.positionY() ,*pos.positionZ());
      //move it according to offset
      current_position -= center;

      //get next step according to position based pid controller
      //Eigen::Vector3d step_vec =  basic_controller.positionBasedStep(current_position, dur);

      //use next step in force based pid controller
      Eigen::Vector3d step_vec = vel_controller.forceBasedStep(current_position, dur);

      //update force vector
      fd_haptic_hw_test::JointArray joint_array;
      *joint_array.jointX() = step_vec.x();
      *joint_array.jointY() = step_vec.y();
      *joint_array.jointZ() = step_vec.z();

      last_time = time;

      dev.write(joint_array);


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_haptic_controller");

  ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("keyboard_input", 1000, keyboardCallback);
  //ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>("device position stamped", 1000);

  fd_haptic_hw_test::FdHapticWrapper dev = init_device();

  if(!dev.isInitialized())return 0;

  if(!dev.setPositionMovementParameters(20.0, 50.0, 50.0)){
    std::cout << "Couldn't set movement parameters" << std::endl;
    return 0;
  }

  double m_force = 14.0;
  dev.setMaximumForce(m_force);

  Eigen::Vector3d center(0.0, 0.0, 0.0);

  fd_haptic_hw_test::JointArray joint_array;

  haptic_controller::PositionBasedController pos_controller;
  haptic_utils::omega_device_limits om_limits;


  double cube_length = 5;


  //init position based controller
  Eigen::Vector3d cube_lims(cube_length, cube_length, cube_length);
  center = om_limits.get_center_device();
  pos_controller.initBoundingBox(center, cube_length, cube_length, cube_length);
  pos_controller.initBoundingCone(center, 5.0, -1.0, 2.0, 4);
  pos_controller.setSpringValues(center, cube_lims, 0.0, 3.0);
  //pos_controller.switchBoundingModell();

  Eigen::Vector3d force_lim(m_force, m_force , m_force);

  Eigen::Vector3d pos_lim = om_limits.get_limits();
  haptic_controller::BasicHapticController vel_controller(pos_lim, force_lim);

  double p_val_x(0.8), i_val_x(0.00), d_val_x(0.00120675), i_min_x(0.0), i_max_x(0.0);
  double p_val_y(0.8), i_val_y(0.00), d_val_y(0.00111525), i_min_y(0.0), i_max_y(0.0);
  double p_val_z(0.8), i_val_z(0.00), d_val_z(0.00120675), i_min_z(0.0), i_max_z(0.0);
  vel_controller.setPIDValuesForceX(p_val_x, i_val_x, d_val_x, i_min_x, i_max_x);
  vel_controller.setPIDValuesForceY(p_val_y, i_val_y, d_val_y, i_min_y, i_max_y);
  vel_controller.setPIDValuesForceZ(p_val_z, i_val_z, d_val_z, i_min_z, i_max_z);
  double damp_ratio = 12.5, mass_spring = 1.0;//over damped
  vel_controller.setSpringValues(center, pos_lim, damp_ratio, m_force, mass_spring);

  dev.sleepSeconds(2);


  ros::Time last_time = ros::Time::now(), last_time_update = ros::Time::now();


  while(ros::ok()){

    ros::Time now = ros::Time::now();
    //usePositionBasedController(dev, pos_controller);
    useVelocityBasedController(dev, vel_controller, last_time);
    last_time = now;

  }

}

