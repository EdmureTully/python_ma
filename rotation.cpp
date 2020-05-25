
#include "stdafx.h"
#include "rotation.h"
#include<vector>
#include <iostream>

#define _USE_MATH_DEFINES
#include<math.h>

std::vector<Eigen::Vector3d> create_points_on_circle(unsigned int number, double radius, double height) {
	double radians = 2 * M_PI / number;
	std::vector<Eigen::Vector3d> pts;

	for (unsigned int i = 0; i < number; i++) {
		double x = std::cos(radians*i) * radius;
		double y = std::sin(radians*i) * radius;
		double z = height;
		pts.push_back(Eigen::Vector3d(x, y, z));
	}

	return pts;
}

std::vector<Eigen::Vector3d> create_points_on_line(const Eigen::Vector3d & start, const Eigen::Vector3d & end, int steps)
{
	Eigen::Vector3d diff = end - start;
	std::vector<Eigen::Vector3d> pts;
	
	for (unsigned int i = 0; i < steps; i++) {
		Eigen::Vector3d pt = start + (1.0 / steps) * i * diff;
		pts.push_back(pt);
	}

	return pts;
}

Eigen::Matrix3d look_at_function(const Eigen::Vector3d& curr_pt, const Eigen::Vector3d& target, const Eigen::Vector3d& up) {
	Eigen::Vector3d forward = curr_pt - target;
	forward.normalize();

	Eigen::Vector3d side = forward.cross(up);
	side.normalize();

	Eigen::Vector3d upward = side.cross(forward);
	upward.normalize();

	Eigen::Matrix3d look_at_matrix;

	look_at_matrix.block<1, 3>(0, 0) = side;
	look_at_matrix.block<1, 3>(1, 0) = upward;
	look_at_matrix.block<1, 3>(2, 0) = -forward;

	return look_at_matrix;
}

double calculate_angle(const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir) {
	Eigen::Vector3d cross = curr_dir.cross(des_dir);
	double cross_norm = cross.norm();

	double dot = curr_dir.dot(des_dir);
	double angle = std::atan(cross_norm / dot);
	
	return angle;
}

Eigen::Vector3d calculate_rotation_axis(const Eigen::Vector3d& curr_dir,const Eigen::Vector3d& des_dir) {
	Eigen::Vector3d rot_axis = curr_dir.cross(des_dir); 
	rot_axis.normalize();

	return rot_axis;
}

Eigen::Matrix3d create_skew_symmetric_matrix(const Eigen::Vector3d& rot_axis) {
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

Eigen::Matrix3d calculate_rodrigues_rotation_matrix(const Eigen::Vector3d& curr_pt, const Eigen::Vector3d& des_pt) {
	double theta = calculate_angle(curr_pt, des_pt);
	Eigen::Vector3d rot_axis = calculate_rotation_axis(curr_pt, des_pt);
	Eigen::Matrix3d skew_sym_matrix = create_skew_symmetric_matrix(rot_axis);
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

	Eigen::Matrix3d rot_mat = identity + std::sin(theta) * skew_sym_matrix + (1 - std::cos(theta)) * skew_sym_matrix * skew_sym_matrix;
	
	return rot_mat;
}

Eigen::Quaterniond calculate_quaternion(const Eigen::Vector3d & curr_vec, const Eigen::Vector3d & des_vec)
{
	double angle = calculate_angle(curr_vec, des_vec);
	Eigen::Vector3d rot_axis = calculate_rotation_axis(curr_vec, des_vec);
	Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, rot_axis));
	quat.normalize();

	return quat;
}
//psy3 metric
double angle_quaternions(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2)
{
	double quat_dot = quat1.dot(quat2);

	return std::acos(std::abs(quat_dot));
}
double distance_quaternions(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2)
{
	double quat_dot = quat1.dot(quat2);

	return (1.0 - std::abs(quat_dot));
}
//psy 1
double distance_euler_angles(const Eigen::Vector3d & angle1, const Eigen::Vector3d & angle2)
{
	double a1 = angle1[0];
	double a2 = angle2[0];
	double b1 = angle1[1];
	double b2 = angle2[1];
	double c1 = angle1[2];
	double c2 = angle2[2];

	double d1 = std::min(std::abs(180.0 / M_PI * (a1 - a2)), 360.0 - std::abs(180.0 / M_PI * (a1 - a2)));
	double d2 = std::min(std::abs(180.0 / M_PI * (b1 - b2)), 360.0 - std::abs(180.0 / M_PI * (b1 - b2)));
	double d3 = std::min(std::abs(180.0 / M_PI * (c1 - c2)), 360.0 - std::abs(180.0 / M_PI * (c1 - c2)));
	
	return std::sqrt(d1*d1 + d2*d2 + d3*d3);
}

//psy5
double rotation_matrix_deviation(const Eigen::Matrix3d & rot_mat1, const Eigen::Matrix3d & rot_mat2)
{

	Eigen::Matrix3d rot_dev = Eigen::Matrix3d::Identity() - rot_mat1 * rot_mat2.transpose();
	return rot_dev.norm();
}

std::vector<double> calculate_relative_difference(const std::vector<double>& values)
{
	std::vector<double> rel_diff;
	std::vector<double>::const_iterator iter(values.begin()), list_end(values.end());

	double last_val(*iter);
	iter++;

	for (; iter != list_end; iter++) {
		rel_diff.push_back(std::abs(last_val - *iter) / std::max(std::abs(last_val), std::abs(*iter)));
		last_val = *iter;
	}

	return rel_diff;
}

roll_dummy::roll_dummy()
{
}

roll_dummy::~roll_dummy()
{
}

void roll_dummy::test_wrap_angle()
{
	std::vector<double> values;

	for (double i = 0.0; i < M_PI; i += 0.1) {
		values.push_back(-i);
	}

	std::vector<double> copy_values = values;

	values.push_back(-M_PI);
	values.insert(values.end(), copy_values.begin(), copy_values.end());
	
	values.push_back(-M_PI);
	values.insert(values.end(), copy_values.begin(), copy_values.end());
	values.push_back(-M_PI);
	values.insert(values.end(), copy_values.begin(), copy_values.end());
	std::vector<double>::iterator iter(values.begin()), end(values.end());
	
	double prev_angle(0.0), angle(0.0);

	for (; iter != end; iter++) {
		angle = *iter;
		double test = wrap_angle(prev_angle, angle);
		Eigen::Matrix3d rot_mat = calc_rot_mat_x(test);
		std::cout << test << std::endl;
		std::cout << "rot matrix" << std::endl;
		std::cout << rot_mat << std::endl;

		prev_angle = angle;
	}

}

double roll_dummy::wrap_angle(double prev_angle, double angle)
{
	double diff = angle - prev_angle;

	if (diff <= -M_PI) {
		if (counter != 0) {
			counter = 0;
		}
		else {
			counter = -1;
		}
	}

	if (diff >= M_PI) {
		if (counter != 0) {
			counter = 0;
		}
		else {
			counter = 1;
		}
	}

	angle = counter * M_PI + angle;

	return angle;
}

Eigen::Matrix3d roll_dummy::calc_rot_mat_x(double angle)
{
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

	rot(1, 1) = std::cos(angle);
	rot(1, 2) = -std::sin(angle);
	rot(2, 1) = std::sin(angle);
	rot(2, 2) = std::cos(angle);

	return rot;
}
