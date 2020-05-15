#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

//own
#include "ros_helpers.h"
#include "bounding_geometry.h"

//other
#include <Eigen/Dense>

//ros
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace target_tracker {

class TargetTracker{
public:

  TargetTracker();

  TargetTracker(Eigen::Vector3d scale);

  tf::StampedTransform get_last_transform()const;

  Eigen::Matrix4d get_lastest_transform_as_matrix()const;

  Eigen::Matrix4d get_current_target_pose()const;

  Eigen::Matrix3d get_desired_rotation_matrix(Eigen::Vector3d position, Eigen::Vector3d desired_position, Eigen::Vector3d center);

  void input_device(const geometry_msgs::Pose::ConstPtr& msg);

  void input_device(const Eigen::Matrix4d& inp_mat);

  void input_pose(geometry_msgs::Pose&  pose);



  void set_panda_ref_tf(const tf::StampedTransform& panda_ref_tf);

  void set_first_rotation(Eigen::Matrix3d rot_mat);

  void set_pivot_point(const Eigen::Vector3d& p_point);

  void init_orientation();

  void init_panda_ref();

  bool set_reference_pose(const Eigen::Matrix4d& input_mat);

  bool check_point_bounding_cone(Eigen::Vector3d& pt);

  bool check_current_point_bounding_cone();

  std::vector<Eigen::Vector3d> circle_trajectory(const Eigen::Vector3d& start_point, double radius, unsigned int number_pts);

  std::vector<Eigen::Vector3d> line_trajectory(const Eigen::Vector3d& end_point, unsigned int number_pts);

  //rotation/orientation stuff
  double calculate_angle(const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir);

  Eigen::Vector3d calculate_rotation_axis(const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir);

  Eigen::Matrix3d create_skew_symmetric_matrix(const Eigen::Vector3d& rot_axis);

  Eigen::Matrix3d calculate_rodrigues_rotation_matrix(const Eigen::Vector3d& curr_pt,const Eigen::Vector3d& des_pt);

  Eigen::Quaterniond calculate_quaternion(const Eigen::Vector3d& curr_vec, const Eigen::Vector3d& des_vec);
  //end rotation/orientation stuff


  void simple_input_rotation(const Eigen::Matrix4d&  mat);

  void simple_input_init_rotation(const Eigen::Matrix4d&  mat);

  Eigen::Matrix4d rotate(const Eigen::Matrix4d& rotation);

  Eigen::Matrix4d init_rotate(const Eigen::Matrix4d& rotation);

  Eigen::Matrix4d get_reference_pose()const;



  std::vector<Eigen::Vector3d> line_trajectory(const Eigen::Vector3d& start, const Eigen::Vector3d& end, unsigned int steps);

  std::vector<geometry_msgs::Point> line_trajectory(const geometry_msgs::Point& start, const geometry_msgs::Point & end, unsigned int steps);

  //std::vector<geometry_msgs::Point> line_trajectory(const tf::Vector3& start, const tf::Vector3& end, unsigned steps);

  std::vector<tf::Vector3> line_trajectory(const tf::Vector3 &start, const tf::Vector3 &end, unsigned int steps);

  std::vector<tf::Vector3> position_trajectory(const tf::Vector3& start, const tf::Vector3& end, const unsigned int steps);

  Eigen::Matrix3d calculate_rotation_to_pivot_point(const Eigen::Vector3d&  p_point, const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir);

  void set_length_shaft(const double shaft);

  void de_init_ref_pose();

  void reset_translation();

  ros::NodeHandle n;

private:

  Eigen::Matrix4d adjust_input(const Eigen::Matrix4d& input);

  Eigen::Matrix4d adjust_to_input(const Eigen::Matrix4d& input);

  Eigen::Matrix4d input_forward(Eigen::Matrix4d input_mat);

  Eigen::Vector3d rotate_around_point(const Eigen::Vector3d& point, const Eigen::Vector3d& point_to_rotate, const Eigen::Matrix3d& rot_matrix);

  void calculate_scale();

  void update_cone();

  Eigen::Matrix4d panda_ref;

  Eigen::Matrix4d reference_pose;

  Eigen::Matrix4d target_pose;

  Eigen::Matrix4d last_transform_matrix = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity();

  Eigen::Matrix3d last_rotation_matrix;

  Eigen::Matrix3d rotation_input = Eigen::Matrix3d::Identity();

  Eigen::Vector3d scale;

  Eigen::Vector3d init_position_device;

  Eigen::Vector3d pivot_point;

  Eigen::Vector3d last_z_axis = Eigen::Vector3d(0,0,0) ;

  Eigen::Vector3d last_translation = Eigen::Vector3d(0,0,0);

  tf::TransformListener tf_listener;

  tf::StampedTransform last_transform;

  ros::Subscriber input_pose_sub;

  geometry::cone bounding_cone;

  geometry::geometry_procedures bounding_procedure;

  bool last_z_axis_init = false;

  bool init_ref_pose;

  double length_shaft;

  double max_z_input_device = 0.15;
};
}

#endif // TARGET_TRACKER_H
