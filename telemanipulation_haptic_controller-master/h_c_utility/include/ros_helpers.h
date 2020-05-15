#ifndef ROS_HELPERS_H
#define ROS_HELPERS_H

#include <Eigen/Dense>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace ros_helpers {

Eigen::Vector3d extract_position(const Eigen::MatrixXd& mat);

Eigen::MatrixXd pose_to_matrix(const geometry_msgs::Pose& pose);

Eigen::Matrix4d pose_stamped_to_matrix(const geometry_msgs::PoseStamped& pose_stamped);

geometry_msgs::PoseStamped matrix_to_posestamped(const Eigen::MatrixXd& mat);

Eigen::MatrixXd tf_to_matrix(const tf::Transform& transform);

tf::Transform matrix_to_tf(const Eigen::Matrix4d& mat);

geometry_msgs::Point eigen_vector_to_geometry_msg_point(const Eigen::Vector3d& point);

Eigen::Vector3d tf_vector_to_eigen_vector(const tf::Vector3& vec);

tf::Vector3 eigen_vector_to_tf_vector(const Eigen::Vector3d& vec);

Eigen::Vector3d geometry_msg_point_to_eigen_vector(const geometry_msgs::Point& msg);

Eigen::Matrix4d three_to_four_d_rotation_matrix(const Eigen::Matrix3d& rot_mat);

}

#endif // ROS_HELPERS_H
