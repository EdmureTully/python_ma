#pragma once

#include <Eigen/Dense>
#include <vector>

std::vector<Eigen::Vector3d> create_points_on_circle(unsigned int number, double radius, double height);

std::vector<Eigen::Vector3d> create_points_on_line(const Eigen::Vector3d& start, const Eigen::Vector3d& end, int steps);

Eigen::Matrix3d look_at_function(const Eigen::Vector3d& curr_pt, const Eigen::Vector3d& target, const Eigen::Vector3d& up);

double calculate_angle(const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir);

Eigen::Vector3d calculate_rotation_axis(const Eigen::Vector3d& curr_dir, const Eigen::Vector3d& des_dir);

Eigen::Matrix3d create_skew_symmetric_matrix(const Eigen::Vector3d& rot_axis);

Eigen::Matrix3d calculate_rodrigues_rotation_matrix(const Eigen::Vector3d& curr_pt,const Eigen::Vector3d& des_pt);

Eigen::Quaterniond calculate_quaternion(const Eigen::Vector3d& curr_vec, const Eigen::Vector3d& des_vec);

//metrics

double angle_quaternions(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2);

double distance_quaternions(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2);

double distance_euler_angles(const Eigen::Vector3d& angle1, const Eigen::Vector3d& angle2);

double rotation_matrix_deviation(const Eigen::Matrix3d& rot_mat1, const Eigen::Matrix3d& rot_mat2);

std::vector<double> calculate_relative_difference(const std::vector<double>& values);

class roll_dummy {
public:
	roll_dummy();
	~roll_dummy();
	
	void test_wrap_angle();



private:

	

	double wrap_angle(double prev_angle, double angle);

	Eigen::Matrix3d calc_rot_mat_x(double angle);

	int counter = 0;
	double prev_angle, angle;
};