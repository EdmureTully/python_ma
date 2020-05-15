#include "ros_helpers.h"
#include <std_msgs/Header.h>

namespace ros_helpers {

Eigen::Vector3d extract_position(const Eigen::MatrixXd& mat)
{
  Eigen::Vector3d position(mat(0,3), mat(1,3), mat(2,3));
  return position;
}

Eigen::MatrixXd pose_to_matrix(const geometry_msgs::Pose& pose)
{
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion;
  quaternion.x() = pose.orientation.x;
  quaternion.y() = pose.orientation.y;
  quaternion.z() = pose.orientation.z;
  quaternion.w() = pose.orientation.w;

  Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();

  Eigen::Matrix4d transformation;
  transformation.setIdentity();
  transformation.block<3,3>(0,0) = rotation;
  transformation.block<3,1>(0,3) = position;

  return transformation;
}

geometry_msgs::PoseStamped matrix_to_posestamped(const Eigen::MatrixXd& mat)
{
  Eigen::Vector3d position = mat.block<3,1>(0,3);
  Eigen::Matrix3d rotation = mat.block<3,3>(0,0);
  Eigen::Quaterniond quaternion (rotation);



  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "panda_link_0";

  pose_stamped.pose.position.x = position.x();
  pose_stamped.pose.position.y = position.y();
  pose_stamped.pose.position.z = position.z();

  pose_stamped.pose.orientation.w = quaternion.w();
  pose_stamped.pose.orientation.x = quaternion.x();
  pose_stamped.pose.orientation.y = quaternion.y();
  pose_stamped.pose.orientation.z = quaternion.z();

  return pose_stamped;
}

Eigen::MatrixXd
tf_to_matrix(const tf::Transform& transform)
{
  Eigen::Vector3d position;
  position.x() = transform.getOrigin().x();
  position.y() = transform.getOrigin().y();
  position.z() = transform.getOrigin().z();

  if(std::abs(position.x()) < 0.0001) position.x()= 0.0001;
  if(std::abs(position.y()) < 0.0001) position.y()= 0.0001;
  if(std::abs(position.z()) < 0.0001) position.z()= 0.0001;


  Eigen::Quaterniond quaternion;
  quaternion.x() = transform.getRotation().x();
  quaternion.y() = transform.getRotation().y();
  quaternion.z() = transform.getRotation().z();
  quaternion.w() = transform.getRotation().w();

  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  Eigen::Matrix4d transformation;
  transformation.setIdentity();
  transformation.block<3,3>(0,0) = rotation;
  transformation.block<3,1>(0,3) = position;

  return transformation;
}

tf::Transform matrix_to_tf(const Eigen::Matrix4d& mat)
{
  //std::cout << mat << std::endl;
  Eigen::Vector3d position = mat.block<3,1>(0,3);
  Eigen::Matrix3d rotation = mat.block<3,3>(0,0);

  Eigen::Quaterniond quaternion(rotation);

  tf::Transform transform;
  tf::Vector3 tf_position;
  tf::Quaternion tf_quaternion;

  tf_position.setX(position.x());
  tf_position.setY(position.y());
  tf_position.setZ(position.z());

//  std::cout<< "X: " << tf_position.getX() << std::endl;
//  std::cout<< "Y: " << tf_position.getY() << std::endl;
//  std::cout<< "Z: " << tf_position.getZ() << std::endl;

  tf_quaternion.setX(quaternion.x());
  tf_quaternion.setY(quaternion.y());
  tf_quaternion.setZ(quaternion.z());
  tf_quaternion.setW(quaternion.w());

  transform.setOrigin(tf_position);
  transform.setRotation(tf_quaternion);

  return transform;
}

geometry_msgs::Point eigen_vector_to_geometry_msg_point(const Eigen::Vector3d &point)
{
  geometry_msgs::Point pt;
  pt.x = point.x();
  pt.y = point.y();
  pt.z = point.z();

  return pt;
}


Eigen::Vector3d geometry_msg_point_to_eigen_vector(const geometry_msgs::Point &msg)
{
  Eigen::Vector3d pt(msg.x, msg.y, msg.z);
  return pt;
}

Eigen::Vector3d tf_vector_to_eigen_vector(const tf::Vector3 &vec)
{
  return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
}

tf::Vector3 eigen_vector_to_tf_vector(const Eigen::Vector3d &vec)
{
  return tf::Vector3(vec.x(), vec.y(), vec.z());
}

Eigen::Matrix4d three_to_four_d_rotation_matrix(const Eigen::Matrix3d& rot_mat)
{
  Eigen::Matrix4d ret_mat = Eigen::Matrix4d::Identity();
  ret_mat.block<3,3>(0,0) = rot_mat;
  return ret_mat;
}

Eigen::Matrix4d pose_stamped_to_matrix(const geometry_msgs::PoseStamped &pose_stamped)
{
  Eigen::Vector3d position(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
  Eigen::Quaterniond quaternion;
  quaternion.x() = pose_stamped.pose.orientation.x;
  quaternion.y() = pose_stamped.pose.orientation.y;
  quaternion.z() = pose_stamped.pose.orientation.z;
  quaternion.w() = pose_stamped.pose.orientation.w;

  Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();

  Eigen::Matrix4d transformation;
  transformation.setIdentity();
  transformation.block<3,3>(0,0) = rotation;
  transformation.block<3,1>(0,3) = position;

  return transformation;
}



}

