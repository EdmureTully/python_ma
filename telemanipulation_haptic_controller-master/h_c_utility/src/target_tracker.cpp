//own
#include "target_tracker.h"

//std
#include <math.h>

namespace target_tracker{

static bool init(false);

TargetTracker::TargetTracker()
{
  init_panda_ref();
  last_rotation_matrix = Eigen::Matrix3d::Identity();
}

TargetTracker::TargetTracker(Eigen::Vector3d scale):
  scale(scale)
{
  last_rotation_matrix = Eigen::Matrix3d::Identity();
  init_panda_ref();
}

Eigen::Matrix4d TargetTracker::get_current_target_pose() const
{
  std::cout << "Current target pose" << std::endl;
  std::cout << target_pose << std::endl;
  return target_pose;
}



tf::StampedTransform TargetTracker::get_last_transform() const
{
  return last_transform;
}

Eigen::Matrix4d TargetTracker::get_lastest_transform_as_matrix() const
{
  return last_transform_matrix;
}

bool TargetTracker::set_reference_pose(const Eigen::Matrix4d& input_mat)
{
  reference_pose = input_mat;
  last_translation = Eigen::Vector3d(0.0, 0.0, 0.0);

  return true;
}

void TargetTracker::input_device(const geometry_msgs::Pose::ConstPtr &msg)
{
  Eigen::Matrix4d input_mat = adjust_input(ros_helpers::pose_to_matrix(*msg));

  if(!init && set_reference_pose(input_mat)){
    init =true;
  }

  target_pose = input_forward(input_mat);

}

void TargetTracker::input_device(const Eigen::Matrix4d &inp_mat)
{
  target_pose = input_forward(inp_mat);
}

void TargetTracker::input_pose(geometry_msgs::Pose &pose)
{
  Eigen::Matrix4d input_mat = adjust_input(ros_helpers::pose_to_matrix(pose));

  if(!init && !set_reference_pose(input_mat)){
    init =true;
  }

  input_mat = Eigen::Matrix4d::Identity();
  target_pose = input_forward(input_mat);
}

void TargetTracker::simple_input_rotation(const Eigen::Matrix4d &mat)
{
  target_pose = rotate(mat);
}

void TargetTracker::simple_input_init_rotation(const Eigen::Matrix4d &mat)
{
  target_pose = init_rotate(mat);
}

void TargetTracker::set_panda_ref_tf(const tf::StampedTransform &panda_ref_tf)
{

  panda_ref = ros_helpers::tf_to_matrix(panda_ref_tf);
}

void TargetTracker::set_first_rotation(Eigen::Matrix3d rot_mat)
{
  last_rotation_matrix = rot_mat;
}

void TargetTracker::set_pivot_point(const Eigen::Vector3d &p_point)
{
  pivot_point = p_point;
}


void TargetTracker::init_panda_ref()
{
  tf::StampedTransform transform;
  bool init_panda_mat(false);

  while(init_panda_mat == false){

    try {
      tf_listener.lookupTransform("/panda_link_0", "/panda_cartesian_control_target",ros::Time(0), transform);
      init_panda_mat = true;
    } catch (tf::TransformException tf_exception) {
      ROS_ERROR("%s", tf_exception.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  panda_ref = ros_helpers::tf_to_matrix(transform);
}

bool TargetTracker::check_point_bounding_cone(Eigen::Vector3d &pt)
{
  bool inside = bounding_procedure.point_inside_object(bounding_cone, pt);
  return inside;
}

bool TargetTracker::check_current_point_bounding_cone()
{
  Eigen::Vector3d pos = target_pose.block<3,1>(0,3);
  bool inside = bounding_procedure.point_inside_object(bounding_cone, pos);
  return inside;
}


Eigen::Matrix4d TargetTracker::input_forward(Eigen::Matrix4d input_mat)
{
  Eigen::Matrix3d rot_part = input_mat.block<3,3>(0,0);

  if(!init_ref_pose){
    set_reference_pose(input_mat);
    init_ref_pose = true;
  }

	//calculate current point of end effector
  Eigen::Matrix4d result = panda_ref *input_mat * reference_pose.inverse();// * reference_pose.inverse();

  Eigen::Vector3d temp = result.block<3,1>(0,3);



  //rotate current rotation by new rotation
  rotation_input = rotation_input * rot_part;

  //Last point of the end effector
  Eigen::Vector3d current_dir =  target_pose.block<3,1>(0,3) ;

  //calculate rotation between the current and the next point so that they face towards the pivot point
  Eigen::Matrix3d rot_mat = calculate_rotation_to_pivot_point(pivot_point,   current_dir, temp);

  //rotate current point to new point
  Eigen::Vector3d next_dir = rotate_around_point(pivot_point, current_dir, rot_mat);

  std::cout << "next dir after rotation" << std::endl;
  std::cout << next_dir << std::endl;

  //update orientation with actual orientation
  rotation_input *= rot_mat;

  result = panda_ref * input_mat * reference_pose.inverse();
  //apply all rotations up to now
  result.block<3,3>(0,0) *= rotation_input;

  return result;
}

Eigen::Vector3d TargetTracker::rotate_around_point(const Eigen::Vector3d &point, const Eigen::Vector3d &point_to_rotate, const Eigen::Matrix3d &rot_matrix)
{
  return rot_matrix  * (point_to_rotate - point)  + point;
}

double TargetTracker::calculate_angle(const Eigen::Vector3d &curr_dir, const Eigen::Vector3d &des_dir)
{
  Eigen::Vector3d cross = curr_dir.cross(des_dir);


  if((curr_dir-des_dir).norm() <= 0.00001)return 0;

  double cross_norm = cross.norm();

  double dot = curr_dir.dot(des_dir);

  if(dot == 0.0) return 0.0;

  double angle = std::atan(cross_norm / dot);

  return angle;
}

Eigen::Vector3d TargetTracker::calculate_rotation_axis(const Eigen::Vector3d &curr_dir, const Eigen::Vector3d &des_dir)
{
  Eigen::Vector3d rot_axis = curr_dir.cross(des_dir);
  rot_axis.normalize();

  return rot_axis;
}

Eigen::Matrix3d TargetTracker::create_skew_symmetric_matrix(const Eigen::Vector3d &rot_axis)
{
  double x = rot_axis.x(), y = rot_axis.y(), z = rot_axis.z();
  Eigen::Matrix3d skew_sym_matrix;
  Eigen::Vector3d r1(0, -z, y);
  Eigen::Vector3d r2(z, 0, -x);
  Eigen::Vector3d r3(-y, x, 0);

  skew_sym_matrix.block<1, 3>(0, 0) = r1;
  skew_sym_matrix.block<1, 3>(1, 0) = r2;
  skew_sym_matrix.block<1, 3>(2, 0) = r3;

  return skew_sym_matrix;
}

Eigen::Matrix3d TargetTracker::calculate_rodrigues_rotation_matrix(const Eigen::Vector3d &curr_pt, const Eigen::Vector3d &des_pt)
{
  double theta = calculate_angle(curr_pt, des_pt);


  if(theta == 0.0)return Eigen::Matrix3d::Identity();

  Eigen::Vector3d rot_axis = calculate_rotation_axis(curr_pt, des_pt);

  if(rot_axis.norm() <= 0) return Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew_sym_matrix = create_skew_symmetric_matrix(rot_axis);
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d rot_mat = identity + std::sin(theta) * skew_sym_matrix + (1 - std::cos(theta)) * skew_sym_matrix * skew_sym_matrix;
  
  return rot_mat;
}

Eigen::Quaterniond TargetTracker::calculate_quaternion(const Eigen::Vector3d &curr_vec, const Eigen::Vector3d &des_vec)
{
  double angle = calculate_angle(curr_vec, des_vec);
  Eigen::Vector3d rot_axis = calculate_rotation_axis(curr_vec, des_vec);

  if(rot_axis.norm() == 0.0) return Eigen::Quaterniond();

  Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, rot_axis));
  quat.normalize();

  return quat;
}


Eigen::Matrix4d TargetTracker::rotate(const Eigen::Matrix4d &rotation)
{

    Eigen::Matrix3d rot_part = rotation.block<3,3>(0,0);

    last_rotation_matrix = last_rotation_matrix * rotation.block<3,3>(0,3);


    last_translation = rotation.block<3,1>(0,3);

    Eigen::Matrix4d copy = rotation;

    copy.block<3,3>(0,0) = last_rotation_matrix;

    Eigen::Matrix4d rotation_result = panda_ref * copy;

    return rotation_result;
}

Eigen::Matrix4d TargetTracker::init_rotate(const Eigen::Matrix4d &rotation)
{

  init_transform = rotation;

  panda_ref = panda_ref * init_transform;

  return panda_ref;
}

Eigen::Matrix4d TargetTracker::get_reference_pose() const
{
  return reference_pose;
}

Eigen::Matrix3d TargetTracker::calculate_rotation_to_pivot_point(const Eigen::Vector3d &p_point, const Eigen::Vector3d &curr_dir, const Eigen::Vector3d &des_dir)
{

  Eigen::Vector3d transformed_curr_dir = curr_dir - p_point;

  transformed_curr_dir.normalize();
  Eigen::Vector3d transformed_des_dir = des_dir - p_point;
  transformed_des_dir.normalize();

  Eigen::Matrix3d rot_mat = calculate_rodrigues_rotation_matrix(transformed_curr_dir, transformed_des_dir);

  return rot_mat;
}

void TargetTracker::set_length_shaft(const double shaft)
{
  length_shaft = shaft;
}

void TargetTracker::de_init_ref_pose()
{
  init_ref_pose = false;
  init_panda_ref();
}

}



