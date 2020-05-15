#ifndef HAPTIC_UTILS_H
#define HAPTIC_UTILS_H


//own


//standard
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <map>
#include <list>

//other
#include <Eigen/Dense>


namespace haptic_utils{

/**
* @brief the sign +/- of a number
*/
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
* @brief writes a given 3 dimensional eigen vector to file
*/
template <typename T>
void write_eigen_vector3_to_console(T& vector){
    std::cout<< "x: " << std::to_string(vector.x()) << ", " << "y: " << std::to_string(vector.y()) << ", " << "z: " << std::to_string(vector.z()) << std::endl;
}

std::list<Eigen::Vector3d> get_linear_trajector(Eigen::Vector3d& start, Eigen::Vector3d& end, int steps);


/**
* @brief The data_writer class writes data to an output file. Is realied as a singleton
*/
class data_writer {
public:

    /**
    * @brief overwrites copy constructor and deletes it. Only one instance necessary.
    */
    data_writer(data_writer const&) = delete;

    /**
    * @brief deletes assignment operator; no other instace possible
    */
    void operator = (data_writer const&) = delete;

    /**
    * @brief get_instace returns the only instance of data_writer
    * @return only instance of data_writer
    */
    static data_writer& get_instance();

    /**
    * @brief set_data_name sets the name/path of the output file
    * @param data_name path/name of the output file
    */
    void set_data_name(std::string& data_name);

    /**
    * @brief get_data_name returns the path/name of the current output file
    * @return path/name of the current output file
    */
    std::string get_data_name() const;

    /**
    * @brief write_string_to_file writes a single string to the output file
    * @param line string to write to the output file
    * @return true if successfull, false otherwise
    */
    bool write_string_to_file(std::string& line);

    /**
    * @brief write_list_to_file writes a list of strings to the output file
    * @param line list of strings to write to the output file
    * @return true if successfull, false otherwise
    */

    bool write_list_to_file(std::list<std::string>& string_list);

    /**
    * @brief writes postion array to file
    */
    //bool write_position_array_to_file(fd_haptic_hw_test::PositionArray& pos);

    /**
    * @brief writes force array to file
    */
    //bool write_force_array_to_file(fd_haptic_hw_test::ForceArray& force);

    /**
    * @brief writes a given 3 dimensional eigen vector to file
    */
    template <typename T>
    bool write_eigen_vector3_to_file(T& vector, const std::string& info = ""){
        if (data_name_.empty()) {
            return false;
        }
        static int counter(1);
        std::ofstream file;
        file.open(data_name_, std::ios::out | std::ios::app);

        if (!file.is_open()) {
            return false;
        }
        std::string p_name = "P" + std::to_string(counter);
        counter++;

        file << info;
        file << "punkt(";
        file << std::to_string(vector.x()) << "|" << std::to_string(vector.y())  << "|" << std::to_string(vector.z()) <<  " \""  << p_name << "\""<< ")";
        //file << "\n";
        //file << "x: " << std::to_string(vector.x()) << ", " << "y: " << std::to_string(vector.y()) << ", " << "z: " << std::to_string(vector.z());
        file << "\n";

        file.close();

        return true;
    }


private:

    std::string data_name_;/**< name/path of the output file*/

    /**
    * @brief Standard constructor; private since there should be only one data_writer
    */
    data_writer() {
        data_name_ = std::string("debug_log.txt");
    }


};

class pidValueHelper{

public:

  /**
  * @brief standard constructor
  */
  pidValueHelper();

  /**
  * @brief inserts point into osc_map and latest_occ_map
  * @param point point to insert
  * @param time time in seconds
  */
  void insertPoint(Eigen::Vector3d& point, double time);

  /**
  * @brief returns average_(oscillation time)
  * @return the average oscillation time
  */
  double getAverageOscilationTime()const;

  /**
  * @brief calculates average osc in via osc_map
  */
  void calculateAverageOscTimes();

private:

std::map<int, std::list<double>> osc_map;/**< contains times. a certain point was reached; points are indexed with an integer */
std::map<int, double> latest_occ_map;/**< latest time a point was reached*/
std::list<Eigen::Vector3d> vec_list;/**< list of all points reached */
double p_value, d_value, i_value;/**< p, i and d values used */
double average;/**< average oscillation time */


};

class omega_device_limits{

public:

  /**
  * @brief standard constructor
  */
  omega_device_limits();

  /**
  * @brief returns the three dimensional center given the device's limits. Center is negative end + (positve end + negative end)/2
  * @return center of the device's workspace
  */
  Eigen::Vector3d get_center_device();

  /**
  * @brief returns the middle between each positive and negative value. Limits should be equidistant from the center
  * @return vector containing limits for each dimension
  */
  Eigen::Vector3d get_limits()const;



private:

    //values were measured using the omega device
    double x_neg_limit = -0.051748;/**< negative x dimension */
    double x_pos_limit = 0.068699;/**< positive x dimension */
    double y_neg_limit = -0.107635;/**< negative y dimension */
    double y_pos_limit = 0.102156;/**< positive y dimension */
    double z_neg_limit = -0.07245;/**< negative z dimension */
    double z_pos_limit = 0.12;/**< positive z dimension */
};


}


#endif // HAPTIC_UTILS_H
