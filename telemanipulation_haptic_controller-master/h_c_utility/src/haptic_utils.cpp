#include "haptic_utils.h"

namespace haptic_utils {


data_writer &data_writer::get_instance()
{

    static data_writer instance;

    return instance;
}

void data_writer::set_data_name(std::string &data_name)
{
    data_name_ = data_name;
}

bool data_writer::write_string_to_file(std::string &line)
{
    if (data_name_.empty()) {
        return false;
    }

    std::ofstream file;
    file.open(data_name_, std::ios::out | std::ios::app);

    if (!file.is_open()) {
        return false;
    }

    file << line;
    file << "\n";

    file.close();

    return true;
}

bool data_writer::write_list_to_file(std::list<std::string> &string_list)
{
    if (data_name_.empty()) {
        return false;
    }

    std::ofstream file;
    file.open(data_name_, std::ios::out | std::ios::app);

    if (!file.is_open()) {
        return false;
    }

    for (const std::string& line : string_list) {
        file << line;
        file << "\n";
    }

    file.close();

    return true;
}

//bool data_writer::write_position_array_to_file(fd_haptic_hw_test::PositionArray &pos)
//{
//  if (data_name_.empty()) {
//      return false;
//  }

//  std::ofstream file;
//  file.open(data_name_, std::ios::out | std::ios::app);

//  if (!file.is_open()) {
//      return false;
//  }

//  file << "Positions-Array" << "\n";
//  file << "Erste Stelle: "  << std::to_string(*pos.positionX()) << "\n";
//  file << "Zweite Stelle: "  << std::to_string(*pos.positionY()) << "\n";
//  file << "Dritte Stelle: "  << std::to_string(*pos.positionZ()) << "\n";

//  file.close();

//  return true;
//}

//bool data_writer::write_force_array_to_file(fd_haptic_hw_test::ForceArray &force)
//{
//  if (data_name_.empty()) {
//      return false;
//  }

//  std::ofstream file;
//  file.open(data_name_, std::ios::out | std::ios::app);

//  if (!file.is_open()) {
//      return false;
//  }

//  file << "Positions-Array" << "\n";
//  file << "Erste Stelle: "  << std::to_string(*force.forceX()) << "\n";
//  file << "Zweite Stelle: "  << std::to_string(*force.forceY()) << "\n";
//  file << "Dritte Stelle: "  << std::to_string(*force.forceZ()) << "\n";

//  file.close();

//  return true;
//}

pidValueHelper::pidValueHelper()
{

}

void pidValueHelper::insertPoint(Eigen::Vector3d &point, double time)
{
  std::list<Eigen::Vector3d>::const_iterator vec_iter(vec_list.begin()), vec_list_end(vec_list.end());
  bool found(false);
  int counter(0);

  for(; vec_iter != vec_list_end; vec_iter++){


    if((*vec_iter - point).norm() < 0.003){
      double last_time = latest_occ_map[counter];
      latest_occ_map[counter] = time;
      osc_map[counter].push_back(time -last_time);
      found = true;
      break;
    }
    counter++;
  }

  if(!found){
    latest_occ_map[counter] = time;
    osc_map[counter] = std::list<double>();
    vec_list.push_back(point);
  }

}

double pidValueHelper::getAverageOscilationTime() const
{
  return average;
}

void pidValueHelper::calculateAverageOscTimes()
{
  double sum(0.0);
  int counter(0);

  for(const std::pair<int, std::list<double>>& osc_pair : osc_map){
    for(const double & dur : osc_pair.second){
      sum += dur;
      counter++;
    }
  }
  average = (sum / counter);
}

omega_device_limits::omega_device_limits()
{

}

Eigen::Vector3d omega_device_limits::get_center_device()
{
  double x_center, y_center, z_center;

  x_center = (std::abs(x_neg_limit) + std::abs(x_pos_limit)) / 2 + x_neg_limit;
  y_center = (std::abs(y_neg_limit) + std::abs(y_pos_limit)) / 2 + y_neg_limit;
  z_center = (std::abs(z_neg_limit) + std::abs(z_pos_limit)) / 2 + z_neg_limit;

  return Eigen::Vector3d(x_center, y_center, z_center);
}

Eigen::Vector3d omega_device_limits::get_limits() const
{
  double x_limit =  (std::abs(x_neg_limit) + std::abs(x_pos_limit)) / 2;
  double y_limit =  (std::abs(y_neg_limit) + std::abs(y_pos_limit)) / 2;
  double z_limit =  (std::abs(z_neg_limit) + std::abs(z_pos_limit)) / 2;

  return Eigen::Vector3d(x_limit, y_limit, z_limit);
}

std::list<Eigen::Vector3d> get_linear_trajector(Eigen::Vector3d &start, Eigen::Vector3d &end, int steps)
{
  std::list<Eigen::Vector3d> traj;
  Eigen::Vector3d dist = end - start;
  double stepsd = static_cast<double>(steps);
  traj.push_back(start);
  for(int i = 1; i <= steps; i++){
    Eigen::Vector3d pt = start + ((1.0/stepsd) * i) * dist;
    traj.push_back(pt);
  }
  return traj;
}

}

