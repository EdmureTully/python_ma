
// eigen_master.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "rotation.h"

#include <iostream>
#include <chrono>
#include <numeric>

void print_list(const std::vector<double>& vlist) {
	for (const double& val : vlist) {
		std::cout << val << " ";
	}
	std::cout << std::endl;
}

std::vector<double> calculate_matrix_orientation_rodrigues(std::vector<Eigen::Vector3d>& pts) {
	Eigen::Vector3d center(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0, 1, 0);
	Eigen::Vector3d curr_vec = pts.front();

	Eigen::Matrix3d look_at_mat = look_at_function(curr_vec, center, up);
	Eigen::Matrix3d last_mat = look_at_mat;

	std::vector<double> differences;

	for (const Eigen::Vector3d& pt : pts) {

		if ((curr_vec - pt).norm() < 0.000010) continue;

		Eigen::Matrix3d rod_rot_mat = calculate_rodrigues_rotation_matrix(curr_vec, pt);
		differences.push_back(rotation_matrix_deviation(last_mat, rod_rot_mat));

		look_at_mat = look_at_mat * rod_rot_mat.transpose();
		last_mat = rod_rot_mat;
		curr_vec = rod_rot_mat * curr_vec;
	}
	return differences;
}

std::vector<double> calculate_quaternion_rotation(std::vector<Eigen::Vector3d>& pts) {
	Eigen::Vector3d center(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0, 1, 0);
	Eigen::Vector3d curr_vec = pts.front();

	Eigen::Matrix3d look_at_mat = look_at_function(curr_vec, center, up);
	Eigen::Quaterniond look_at_quat(look_at_mat);
	look_at_quat.normalize();
	Eigen::Quaterniond last_quat = look_at_quat;

	std::vector<double> differences;

	for (const Eigen::Vector3d& pt : pts) {

		if ((curr_vec - pt).norm() < 0.000010) continue;

		Eigen::Quaterniond rot_quat = calculate_quaternion(curr_vec, pt);

		differences.push_back(angle_quaternions(last_quat, rot_quat));
		//std::cout << "Quaternion w" << rot_quat.w() << std::endl;
		//std::cout <<"Orientation" << rot_quat.vec() << std::endl;
		curr_vec = rot_quat * curr_vec;
		look_at_quat = look_at_quat * rot_quat.conjugate();
		last_quat = rot_quat;
	}

	return differences;
}

std::vector<double> calculate_euler_rotation(std::vector<Eigen::Vector3d>& pts) {
	Eigen::Vector3d center(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0, 1, 0);
	Eigen::Vector3d curr_vec = pts.front();

	Eigen::Matrix3d look_at_mat = look_at_function(curr_vec, center, up);
	Eigen::Quaterniond look_at_quat(look_at_mat);
	look_at_quat.normalize();
	Eigen::Quaterniond last_quat = look_at_quat;
	Eigen::Matrix3d last_mat = look_at_mat;
	Eigen::Vector3d last_euler_angles(0.0, 0.0, 0.0);
	std::vector<double> differences;
	for (const Eigen::Vector3d& pt : pts) {

		if ((curr_vec - pt).norm() < 0.000010) continue;

		Eigen::Quaterniond rot_quat = calculate_quaternion(curr_vec, pt);
		Eigen::Vector3d euler_angles = rot_quat.toRotationMatrix().eulerAngles(2, 1, 0);
		Eigen::AngleAxisd roll_angle(euler_angles[0], Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd yaw_angle(euler_angles[1], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitch_angle(euler_angles[2], Eigen::Vector3d::UnitX());
		rot_quat = roll_angle * yaw_angle * pitch_angle;

		Eigen::Matrix3d euler_rot_mat = rot_quat.matrix();
		differences.push_back(distance_euler_angles(last_euler_angles, euler_angles));
		last_euler_angles = euler_angles;
		curr_vec = euler_rot_mat * curr_vec;
		look_at_mat = look_at_mat * euler_rot_mat.transpose();
		last_quat = rot_quat;
	}
	return differences;
}


int main()
{
	roll_dummy rd;
	rd.test_wrap_angle();
    return 0;
}

