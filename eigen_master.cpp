
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
	//std::vector<Eigen::Vector3d> pts = create_points_on_circle(15, 1.0, 1.0);
	Eigen::Vector3d start(1.0, 1.0, 1.0), end(-1.0, 1.0, 1.0);
	std::vector<Eigen::Vector3d> pts = create_points_on_line(start, end, 10);
	unsigned int runs = 1;
	std::cout << "Quaternion rotation" << std::endl;
	std::vector<long> times;
	for (unsigned int i = 0; i < runs; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		std::vector<double> diff = calculate_quaternion_rotation(pts);
		diff = calculate_relative_difference(diff);
		print_list(diff);
		long avg_dist = std::accumulate(diff.begin(), diff.end(), 0.0) / static_cast<long> (diff.size());
		std::cout << "Average rot distance:" << std::endl;
		std::cout << avg_dist<< std::endl;		
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		//std::cout << "Time needed:" << std::endl;
		//std::cout << duration.count() << std::endl;
		times.push_back(duration.count());
	}

	long average = std::accumulate(times.begin(), times.end(), 0) / times.size();
	std::cout << "Average time quaternions:" << std::endl;
	std::cout << average << std::endl;
	times.clear();


	std::cout << "Matrix rotation" << std::endl;
	for (unsigned int i = 0; i < runs; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		std::vector<double> diff = calculate_matrix_orientation_rodrigues(pts);
		diff = calculate_relative_difference(diff);
		print_list(diff);
		long avg_dist = std::accumulate(diff.begin(), diff.end(), 0.0) / diff.size();
		std::cout << "Average rot distance:" << std::endl;
		std::cout << avg_dist << std::endl;
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		//std::cout << "Time needed:" << std::endl;
		//std::cout << duration.count() << std::endl;
		times.push_back(duration.count());
	}
	average = std::accumulate(times.begin(), times.end(), 0) / times.size();
	std::cout << "Average time matrix rotation:" << std::endl;
	std::cout << average << std::endl;

	std::cout << "Euler matrix rotation" << std::endl;
	for (unsigned int i = 0; i < runs; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		std::vector<double> diff = calculate_euler_rotation(pts);
		diff = calculate_relative_difference(diff);
		print_list(diff);
		long avg_dist = std::accumulate(diff.begin(), diff.end(), 0.0) / diff.size();
		std::cout << "Average rot distance:" << std::endl;
		std::cout << avg_dist << std::endl;
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		//std::cout << "Time needed:" << std::endl;
		//std::cout << duration.count() << std::endl;
		times.push_back(duration.count());
	}
	average = std::accumulate(times.begin(), times.end(), 0) / times.size();
	std::cout << "Average time euler:" << std::endl;
	std::cout << average << std::endl;
    return 0;
}

