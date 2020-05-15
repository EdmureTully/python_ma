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
//  tf::StampedTransform transform;

//  while(init == false){

//    try {
//      //ROS_INFO("Try to load transformation from panda link 0 to panda end effector for target tracker.");
//      tf_listener.lookupTransform( "/panda_link_0","/panda_cartesian_control_target", ros::Time(0), transform);
//      //ROS_INFO("Transformation from panda link 0 to panda end effector was loaded for target tracker");
//      init = true;
//    } catch (tf::TransformException tf_exception) {
//      ROS_ERROR("%s", tf_exception.what());
//      ros::Duration(1.0).sleep();
//      continue;
//    }
//  }


//  Eigen::Matrix4d panda_matrix = ros_helpers::tf_to_matrix(transform);
//  last_transform_matrix = panda_matrix;


//  panda_ref = panda_matrix;
  reference_pose = input_mat;
  last_translation = Eigen::Vector3d(0.0, 0.0, 0.0);
  calculate_scale();

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

void TargetTracker::init_orientation()
{

  bool tf_found(false);
  tf::StampedTransform transform_panda_link0_lap_box;
  tf::StampedTransform transform_eef_lap_box;


  while(!tf_found){

    try {

      //ROS_INFO("Try to load transformation from panda link 0 to lap box link for target tracker.");
      tf_listener.lookupTransform( "/panda_link_0","/lap_box_link", ros::Time(0), transform_panda_link0_lap_box);
      //ROS_INFO("Transformation from panda link 0 to lap box link was loaded for target tracker");

      //ROS_INFO("Try to load transformation from panda cartesian control target to lap box link for target tracker.");
      tf_listener.lookupTransform( "/panda_link_0","/panda_cartesian_control_target", ros::Time(0), transform_eef_lap_box);
      //ROS_INFO("Transformation from panda cartesian control target to lap box link was loaded for target tracker");

      tf_found = true;
    } catch (tf::TransformException tf_exception) {
      ROS_ERROR("%s", tf_exception.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }


  tf::Vector3 eef_box_origin = transform_eef_lap_box.getOrigin();
  std::string origin = "x:" + std::to_string(eef_box_origin.getX()) + ", y: " + std::to_string(eef_box_origin.getY()) + ", z: " + std::to_string(eef_box_origin.getZ());
  ROS_INFO("Origin of transformation base 0 to lap box: %s", origin.c_str());

  tf::Vector3 z_axis = tf::Vector3(0.0, 0.0, 1.0) ;



  tf::Vector3 rot_z_axis = transform_eef_lap_box.getBasis() * z_axis;

  std::string rot_origin = "x:" + std::to_string(rot_z_axis.getX()) + ", y: " + std::to_string(rot_z_axis.getY()) + ", z: " + std::to_string(rot_z_axis.getZ());
  ROS_INFO("Z axis of cartesian control target: %s", rot_origin.c_str());

}

void TargetTracker::init_panda_ref()
{
  tf::StampedTransform transform;
  bool init_panda_mat(false);

  while(init_panda_mat == false){

    try {
      //ROS_INFO("Try to load transformation from panda link 0 to panda end effector for target tracker.");
      tf_listener.lookupTransform("/panda_link_0", "/panda_cartesian_control_target",ros::Time(0), transform);
      //ROS_INFO("Transformation from panda link 0 to panda end effector was loaded for target tracker");
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

std::vector<Eigen::Vector3d> TargetTracker::circle_trajectory(const Eigen::Vector3d &start_point, double radius, unsigned int number_pts)
{

  double radians = 2 * M_PI / number_pts;
  std::vector<Eigen::Vector3d> pts;

  double last_x(0.0), last_y(0.0), last_z(0.0);



  for (unsigned int i = 0; i < number_pts; i++) {
    double x = -last_x + std::cos(radians*i) * radius;
    double y = -last_y + std::sin(radians*i) * radius;
    double z = 0.0;
    last_x = 0.0;
    last_y = 0.0;
    last_z = 0.0;

    pts.push_back(Eigen::Vector3d(x, y, z));
  }

  return pts;

}

std::vector<Eigen::Vector3d> TargetTracker::line_trajectory(const Eigen::Vector3d& start_point, const Eigen::Vector3d &end_point, unsigned int number_pts)
{

  std::vector<Eigen::Vector3d> pts;

  Eigen::Vector3d diff = end_point -start_point;

  for (unsigned int i = 1; i <= number_pts; i++) {
    double x = start_point.x() + 1.0/number_pts * diff.x() *i;
    double y = start_point.y() + 1.0/number_pts * diff.y() *i;
    double z = start_point.z() + 1.0/number_pts * diff.z() *i;
    pts.push_back(Eigen::Vector3d(x, y, z));
  }

  return pts;

}

static bool test2 = false;

Eigen::Matrix4d TargetTracker::input_forward(Eigen::Matrix4d input_mat)
{
  Eigen::Matrix3d rot_part = input_mat.block<3,3>(0,0);

  if(!init_ref_pose){
    set_reference_pose(input_mat);
    init_ref_pose = true;
  }

  std::cout << "input mat" << std::endl;
  std::cout << input_mat * reference_pose.inverse() << std::endl;

  Eigen::Matrix4d result = panda_ref *input_mat * reference_pose.inverse();// * reference_pose.inverse();

  Eigen::Vector3d temp = result.block<3,1>(0,3);
  std::cout << "temp of result" << std::endl;
  std::cout << temp << std::endl;



  //input_mat = adjust_to_input(input_mat);
  std::cout << "adjusted input mat" << std::endl;
  std::cout << input_mat << std::endl;

  //rotate current rotation by new rotation
  rotation_input = rotation_input * rot_part;

  //apply current rotation to the input
  last_transform_matrix.block<3,3>(0,0) = last_rotation_matrix;


  //Last point of the end effector
  Eigen::Vector3d current_dir =  target_pose.block<3,1>(0,3) ;
  //current_dir.normalize();
  std::cout << "current dir" << std::endl;
  std::cout << current_dir << std::endl;


  //calculate rotation between the current and the next point so that they face towards the pivot point
  Eigen::Matrix3d rot_mat = calculate_rotation_to_pivot_point(pivot_point,   current_dir, temp);


  Eigen::Vector3d next_dir = rotate_around_point(pivot_point, current_dir, rot_mat);

  std::cout << "next dir after rotation" << std::endl;
  std::cout << next_dir << std::endl;

  rotation_input *= rot_mat;
  //input_mat.block<3,3>(0,0) = rotation_input;
  result = panda_ref * input_mat * reference_pose.inverse();
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

  //angle = std::acos(dot/(des_dir.norm() * curr_dir.norm()));

  std::cout << "Current direction:" << std::endl;
  std::cout << curr_dir << std::endl;
  std::cout << "Desired direction:" << std::endl;
  std::cout << des_dir << std::endl;
  std::cout << "Cross product" << std::endl;
  std::cout << cross << std::endl;
  std::cout << "dot product" << std::endl;
  std::cout << dot << std::endl;
  std::cout << "angle" << std::endl;
  std::cout << angle << std::endl;

  return angle;
}

Eigen::Vector3d TargetTracker::calculate_rotation_axis(const Eigen::Vector3d &curr_dir, const Eigen::Vector3d &des_dir)
{
  Eigen::Vector3d rot_axis = curr_dir.cross(des_dir);
  rot_axis.normalize();
  std::cout << "rotation axis: " << std::endl;
  std::cout << rot_axis << std::endl;

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

  std::cout << "In rodrigues rotation:" << std::endl;
  std::cout << "current point" << std::endl;
  std::cout << curr_pt << std::endl;
  std::cout << "desired point" << std::endl;
  std::cout << des_pt << std::endl;


  if(theta == 0.0)return Eigen::Matrix3d::Identity();

  Eigen::Vector3d rot_axis = calculate_rotation_axis(curr_pt, des_pt);

  if(rot_axis.norm() <= 0) return Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew_sym_matrix = create_skew_symmetric_matrix(rot_axis);
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d rot_mat = identity + std::sin(theta) * skew_sym_matrix + (1 - std::cos(theta)) * skew_sym_matrix * skew_sym_matrix;

//  std::cout << "Rodrigues rotation matrix" << std::endl;
//  std::cout << rot_mat << std::endl;

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

static bool test = false;

Eigen::Matrix4d TargetTracker::rotate(const Eigen::Matrix4d &rotation)
{

    //vielleciht zweite Matrize transponieren

    std::cout << "current target pose" << std::endl;
    std::cout << target_pose << std::endl;

    std::cout << "Inpute rotate: " << std::endl;
    std::cout << rotation << std::endl;

    Eigen::Matrix3d rot_part = rotation.block<3,3>(0,0);

    last_rotation_matrix = last_rotation_matrix * rotation.block<3,3>(0,3);

    std::cout << "last_rot:" << std::endl;
    std::cout << last_rotation_matrix  << std::endl;
//    std::cout << "panda ref bevor rotation:" << std::endl;
//    std::cout << panda_ref << std::endl;


    //last_rotation_matrix = last_rotation_matrix * rot_part;


    last_translation = rotation.block<3,1>(0,3);

    Eigen::Matrix4d copy = rotation;

    copy.block<3,3>(0,0) = last_rotation_matrix;

   // copy.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d current_dir = target_pose.block<3,1>(0,3);

    //hier weitermachen: Möglichkeiten copy vor Multiplikation mmit rodrigues multiplizieren

    Eigen::Matrix4d rotation_result = panda_ref * copy;

    test = true;



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



std::vector<geometry_msgs::Point> TargetTracker::line_trajectory(const geometry_msgs::Point &start, const geometry_msgs::Point &end, unsigned int steps)
{
  geometry_msgs::Point diff;
  diff.x = end.x - start.x;
  diff.y = end.y - start.y;
  diff.z = end.z - start.z;

  std::vector<geometry_msgs::Point> pts;

  for(unsigned int i = 0; i <= steps; i++){
    geometry_msgs::Point pt = start;
    pt.x += (1.0 / steps) * i * diff.x;
    pt.y += (1.0 / steps) * i * diff.y;
    pt.z += (1.0 / steps) * i * diff.z;

    pts.push_back(pt);
  }

  return pts;

}

std::vector<tf::Vector3> TargetTracker::line_trajectory(const tf::Vector3 &start, const tf::Vector3 &end, unsigned int steps)
{
  tf::Vector3 diff;
  diff = end - start;

  std::vector<tf::Vector3> pts;
  std::vector<tf::Vector3> ret_pts;
  for(unsigned int i = 0; i <= steps; i++){
    tf::Vector3 pt(start + (1.0 / steps) * i * diff);
    pts.push_back(pt);
  }

  std::vector<tf::Vector3>::iterator iter(pts.end()), vec_end(pts.begin());

  for(; iter != vec_end; iter--){
    ret_pts.push_back(*iter);
  }

  return ret_pts;
}

std::vector<tf::Vector3> TargetTracker::position_trajectory(const tf::Vector3 &start, const tf::Vector3 &end, const unsigned int steps)
{
  tf::Vector3 diff =  end - start;
  //diff.setZ(0);

  std::cout << "Start Trajektorie" << std::endl;
  std::cout << ros_helpers::tf_vector_to_eigen_vector(start) << std::endl;

  std::cout << "Ende Trajektorie" << std::endl;
  std::cout << ros_helpers::tf_vector_to_eigen_vector(end);

  std::cout << "Differenz der beiden Punkte" << std::endl;
  std::cout << ros_helpers::tf_vector_to_eigen_vector(diff) << std::endl;

  std::vector<tf::Vector3> pts;

  for(unsigned int i = 0; i <=  steps; i++){
    tf::Vector3 pt(start - (1.0 / steps) * i * diff);
    pts.push_back(pt);
  }
  return pts;
}

Eigen::Matrix3d TargetTracker::calculate_rotation_to_pivot_point(const Eigen::Vector3d &p_point, const Eigen::Vector3d &curr_dir, const Eigen::Vector3d &des_dir)
{
  Eigen::Vector3d transformed_p_point = p_point - p_point;


//  std::cout << "current direction" << std::endl;
//  std::cout << curr_dir << std::endl;

//  std::cout << "next point" << std::endl;
//  std::cout << des_dir << std::endl;


  Eigen::Vector3d transformed_curr_dir = curr_dir - p_point;//- p_point;
//  std::cout << "transformed current dir" << std::endl;
//  std::cout << transformed_curr_dir << std::endl;

//  transformed_curr_dir.normalize();
  Eigen::Vector3d transformed_des_dir = des_dir - p_point;// - p_point;
//  std::cout << "transformed des dir" << std::endl;
//  std::cout << transformed_des_dir << std::endl;
//  transformed_des_dir.normalize();

  Eigen::Matrix3d rot_mat = calculate_rodrigues_rotation_matrix(transformed_curr_dir, transformed_des_dir);

  std::cout << "rotation matrix" << std::endl;
  std::cout << rot_mat << std::endl;

//  Eigen::Quaterniond quat = calculate_quaternion(transformed_curr_dir, transformed_des_dir);
//  rot_mat = quat.toRotationMatrix();

//  std::cout << "rotation quat matrix" << std::endl;
//  std::cout << rot_mat << std::endl;

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

void TargetTracker::reset_translation()
{
  last_transform_matrix.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, 0.0);
  last_rotation_matrix = Eigen::Matrix3d::Identity();
  last_transform_matrix = Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d TargetTracker::adjust_input(const Eigen::Matrix4d& input)
{
  Eigen::Matrix4d copy = input;


  copy(1,3) =   input(1,3);
  copy(2,3) = -1.5 * length_shaft / max_z_input_device *  input(2,3);
  return copy;
}

Eigen::Matrix4d TargetTracker::adjust_to_input(const Eigen::Matrix4d &input)
{
  Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
  ret.block<3,3>(0,0) = init_transform.block<3,3>(0,0) * input.block<3,3>(0,0);
  ret.block<3,1>(0,3) = input.block<3,1>(0,3) + init_transform.block<3,1>(0,3);
  return ret;
}


void TargetTracker::calculate_scale()
{
  double z_min = -0.06796;
  double z_dist = std::abs(z_min - reference_pose(2,3));

  scale.z() = panda_ref(2,3)/z_dist;
}

void TargetTracker::update_cone()
{
  Eigen::Vector3d tip(0.0, 0.0, 0.0);
  //height is z dimension
  double height = last_transform_matrix(2,3);
  bounding_cone = geometry::cone(tip, height, 60.0, -1.0, 100);
}

}



