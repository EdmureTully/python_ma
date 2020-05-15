#ifndef HAPTIC_WRAPPER_CPP
#define HAPTIC_WRAPPER_CPP


#include "haptic_wrapper.h"

// Force dimension Haptic SDK
#include "dhdc.h"
// Force dimension Robotic SDK (needed for calibration)


#include <stdexcept>
#include <string>

namespace fd_haptic_hw_test {

JointArray::JointArray()
{
  m_joint_values.fill(0);
}

double* JointArray::jointValues()
{
  return &m_joint_values[0];
}

double* JointArray::atIndex(size_t i)
{
  return &m_joint_values[i];
}

double* JointArray::jointX()
{
  return &m_joint_values[0];
}

double* JointArray::jointY()
{
  return &m_joint_values[1];
}

double* JointArray::jointZ()
{
  return &m_joint_values[2];
}

double* JointArray::jointEx()
{
  return &m_joint_values[3];
}

double* JointArray::jointEy()
{
  return &m_joint_values[4];
}

double* JointArray::jointEz()
{
  return &m_joint_values[5];
}

double* JointArray::jointGripper()
{
  return &m_joint_values[6];
}

FdStatus::FdStatus()
{
  m_status.fill(0);
}

int* FdStatus::statusArray()
{
  return &m_status[0];
}

bool FdStatus::power() const
{
  return m_status[DHD_STATUS_POWER];
}

bool FdStatus::connected() const
{
  return m_status[DHD_STATUS_CONNECTED];
}

bool FdStatus::started() const
{
  return m_status[DHD_STATUS_STARTED];
}

bool FdStatus::reset() const
{
  return m_status[DHD_STATUS_RESET];
}

bool FdStatus::idle() const
{
  return m_status[DHD_STATUS_IDLE];
}

bool FdStatus::force() const
{
  return m_status[DHD_STATUS_FORCE];
}

bool FdStatus::brake() const
{
  return m_status[DHD_STATUS_BRAKE];
}

bool FdStatus::torque() const
{
  return m_status[DHD_STATUS_TORQUE];
}

bool FdStatus::wrist_detected() const
{
  return m_status[DHD_STATUS_WRIST_DETECTED];
}

bool FdStatus::error() const
{
  return m_status[DHD_STATUS_ERROR];
}

bool FdStatus::gravity() const
{
  return m_status[DHD_STATUS_GRAVITY];
}

bool FdStatus::timeguard() const
{
  return m_status[DHD_STATUS_TIMEGUARD];
}

bool FdStatus::wrist_init() const
{
  return m_status[DHD_STATUS_WRIST_INIT];
}

bool FdStatus::redundancy() const
{
  return m_status[DHD_STATUS_REDUNDANCY];
}

bool FdStatus::forceoffcause() const
{
  return m_status[DHD_STATUS_FORCEOFFCAUSE];
}

std::string FdStatus::getStatusText() const
{
  std::string status = "====================";
  status += "\nFd Haptic device status";
  status += "\n--------------------";
  status += std::string("\nPOWER              ") + (power() ? "1" : "0");
  status += std::string("\nCONNECTED          ") + (connected() ? "1" : "0");
  status += std::string("\nSTARTED            ") + (started() ? "1" : "0");
  status += std::string("\nRESET              ") + (reset() ? "1" : "0");
  status += std::string("\nIDLE               ") + (idle() ? "1" : "0");
  status += std::string("\nFORCE              ") + (force() ? "1" : "0");
  status += std::string("\nBRAKE              ") + (brake() ? "1" : "0");
  status += std::string("\nTORQUE             ") + (torque() ? "1" : "0");
  status += std::string("\nWRIST_DETECTED     ") + (wrist_detected() ? "1" : "0");
  status += std::string("\nERROR              ") + (error() ? "1" : "0");
  status += std::string("\nGRAVITY            ") + (gravity() ? "1" : "0");
  status += std::string("\nTIMEGUARD          ") + (timeguard() ? "1" : "0");
  status += std::string("\nWRIST_INIT         ") + (wrist_init() ? "1" : "0");
  status += std::string("\nREDUNDANCY         ") + (redundancy() ? "1" : "0");
  status += std::string("\nFORCEOFFCAUSE      ") + (forceoffcause() ? "1" : "0");
  status += "\n====================";

  return status;
}

FdDeviceType::FdDeviceType(FdDeviceModel model, FdDeviceSide side)
  : m_model{model}
  , m_side{side}
{
}

FdDeviceModel FdDeviceType::getModel() const
{
  return m_model;
}

FdDeviceSide FdDeviceType::getSide() const
{
  return m_side;
}

std::string FdDeviceType::getDescription() const
{
  // We know these are valid enum values, so no further error checking necessary
  std::string description = "";

  switch (m_model)
  {
    case FdDeviceModel::OMEGA_7:
      description += "omega.7";
      break;
    case FdDeviceModel::SIGMA_7:
      description += "sigma.7";
      break;
  }

  description += " (";
  switch (m_side)
  {
    case FdDeviceSide::RIGHT:
      description += "right";
      break;
    case FdDeviceSide::LEFT:
      description += "left";
      break;
  }
  description += ")";

  return description;
}

int FdDeviceType::getTypeForSDK() const
{
  switch (m_model)
  {
    case FdDeviceModel::OMEGA_7:
      switch (m_side)
      {
        case FdDeviceSide::LEFT:
          return DHD_DEVICE_OMEGA331_LEFT;
        case FdDeviceSide::RIGHT:
          return DHD_DEVICE_OMEGA331;
      }

    case FdDeviceModel::SIGMA_7:
      switch (m_side)
      {
        case FdDeviceSide::LEFT:
          return DHD_DEVICE_SIGMA331_LEFT;
        case FdDeviceSide::RIGHT:
          return DHD_DEVICE_SIGMA331;
      }
  }
}

FdHapticWrapper::FdHapticWrapper(FdDeviceType &type)
{
    dev_init = init_device(type);

    enable();


    //performCalibration(true);

}

void FdHapticWrapper::hold()
{
  int message = drdHold();
  if(message){
    printf ("Error: could not set hold position: (%s)\n", dhdErrorGetLastStr ());
  }
}

void FdHapticWrapper::lock(unsigned char mask, bool init)
{
  int message = drdLock(mask, init);
  if(message){
    printf ("Error: could not set lock position: (%s)\n", dhdErrorGetLastStr ());
  }
}

FdHapticWrapper::~FdHapticWrapper()
{
  drdClose(m_device_id);
}

void FdHapticWrapper::enable()
{
  if (dhdSetBrakes(DHD_OFF, m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not disable breaks (error: ") + getLastError() +
                             "). Shutting down.");
  }

  if (dhdEnableForce(DHD_ON, m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not enable force mode for device(error: ") +
                             getLastError() + "). Shutting down.");
  }

  if (dhdSetGravityCompensation(DHD_ON, m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not enable gravity compensation(error: ") +
                             getLastError() + "). Shutting down.");
  }

  if(dhdEnableExpertMode()){
    throw std::runtime_error(std::string("Could not enable expert mode(error: ") +
                             getLastError() + "). Shutting down.");
  }

  readDeviceStatus();
}

void FdHapticWrapper::read(JointArray& pos_target, JointArray& vel_target, JointArray& eff_target)
{
  int result = 0;
  /*
   * Read position
   */
  result += dhdGetPositionAndOrientationRad(pos_target.jointX(),
                                            pos_target.jointY(),
                                            pos_target.jointZ(),
                                            pos_target.jointEx(),
                                            pos_target.jointEy(),
                                            pos_target.jointEz(),
                                            m_device_id);
  result += dhdGetGripperGap(pos_target.jointGripper(), m_device_id);

  /*
   * Read velocity
   */
  result += dhdGetLinearVelocity(
    vel_target.jointX(), vel_target.jointY(), vel_target.jointZ(), m_device_id);
  result += dhdGetAngularVelocityRad(
    vel_target.jointEx(), vel_target.jointEy(), vel_target.jointEz(), m_device_id);
  result += dhdGetGripperLinearVelocity(vel_target.jointGripper(), m_device_id);

  /*
   * Read effort
   */
  result += dhdGetForceAndTorqueAndGripperForce(eff_target.jointX(),
                                                eff_target.jointY(),
                                                eff_target.jointZ(),
                                                eff_target.jointEx(),
                                                eff_target.jointEy(),
                                                eff_target.jointEz(),
                                                eff_target.jointGripper(),
                                                m_device_id);

  // Error handling
  if (result < 0)
  {
    throw std::runtime_error(std::string("Could not read current device state  los(error: ") +
                             getLastError() + "). Shutting down.");
  }
}

void FdHapticWrapper::write(JointArray& eff_cmd)
{
  if (dhdSetForceAndTorqueAndGripperForce(*eff_cmd.jointX(),
                                          *eff_cmd.jointY(),
                                          *eff_cmd.jointZ(),
                                          *eff_cmd.jointEx(),
                                          *eff_cmd.jointEy(),
                                          *eff_cmd.jointEz(),
                                          *eff_cmd.jointGripper(),
                                          m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not set force target (error: ") + getLastError() +
                             "). Shutting down.");
  }
}

void FdHapticWrapper::moveToPosition(JointArray &pos_target)
{
    double position[DHD_MAX_DOF] = {*pos_target.jointX(),
                                   *pos_target.jointY(),
                                   *pos_target.jointZ(),
                                   *pos_target.jointEx(),
                                   *pos_target.jointEy(),
                                   *pos_target.jointEz(),
                                   *pos_target.jointGripper(),
                                    0.0};



    if(drdMoveTo(position)< 0){
        printf ("Error: could not move to position: (%s)\n", dhdErrorGetLastStr ());
        std::string code = std::to_string(dhdErrorGetLast());
        printf("Error: could not move to position: (%s)\n", code.c_str());
    }
}

bool FdHapticWrapper::moveToPosition(PositionArray &pos_target)
{
    if(drdMoveToPos(*pos_target.positionX(), *pos_target.positionY(), *pos_target.positionZ(), true)){
        printf ("Error: could not move to position: (%s)\n", dhdErrorGetLastStr ());
        return false;
    }
    return true;
}

void FdHapticWrapper::reset()
{
    int message = dhdReset();

    if(message){
        printf ("Error: could not move to position: (%s)\n", dhdErrorGetLastStr ());
    }
}

void FdHapticWrapper::setMaximumForce(double force)
{
    int message = dhdSetMaxForce(force);
    if(message){
        printf ("Error: could not set maximum force: (%s)\n", dhdErrorGetLastStr ());
    }
}

void FdHapticWrapper::disableForce()
{
    force_activated = false;
}

void FdHapticWrapper::enableForce()
{
    force_activated = true;
}

bool FdHapticWrapper::forceActive()
{
    return force_activated;
}

void FdHapticWrapper::sleepSeconds(double seconds)
{
  dhdSleep(seconds);
}

void FdHapticWrapper::getCurrentForce(ForceArray &force)
{
  if(dhdGetForce(force.forceX(), force.forceY(), force.forceZ())){
    printf ("Error: could not get cartesian force: (%s)\n", dhdErrorGetLastStr ());
  }
}

bool FdHapticWrapper::setCartesianForce(ForceArray &force)
{

    int message = dhdSetForce(*force.forceX(), *force.forceY(), *force.forceZ());

    if(message && (message == DHD_MOTOR_SATURATED)){
        printf ("Error: could not set cartesian force: (%s)\n", dhdErrorGetLastStr ());
        return false;
    }
    return true;
}

bool FdHapticWrapper::getPositionMoveParameter(PositionArray &pos_array)
{
  if(drdGetPosMoveParam(pos_array.positionX(), pos_array.positionY(), pos_array.positionZ())){
    printf ("Error: could get position move parameters: (%s)\n", dhdErrorGetLastStr ());
    return false;
  }
  return true;
}

bool FdHapticWrapper::isInitialized() const
{
  return dev_init;
}

bool FdHapticWrapper::setPositionMovementParameters(double a_max, double v_max, double jerk)
{
  if(drdSetPosMoveParam(a_max, v_max, jerk)){
    printf ("Error: could not get cartesian positin: (%s)\n", dhdErrorGetLastStr ());
    return false;
  }

  return true;
}

double FdHapticWrapper::getMaximumForce()
{
    return dhdGetMaxForce();
}

PositionArray FdHapticWrapper::getPosition()
{
    PositionArray position;

    int message = dhdGetPosition(position.positionX(), position.positionY(), position.positionZ());

    if(message && !(message == DHD_TIMEGUARD) ){
        printf ("Error: could not get cartesian positin: (%s)\n", dhdErrorGetLastStr ());
    }
    return position;
}

ForceArray FdHapticWrapper::getForce()
{
  ForceArray force;

  int message = dhdGetForce(force.forceX(), force.forceY(), force.forceZ());

  if(message){

  }
}

bool FdHapticWrapper::requiresCalibration() const
{
  return m_status.reset();
}

bool FdHapticWrapper::init_device(FdDeviceType& type)
{
    int device_count = dhdGetDeviceCount();

    if(device_count <= 0){
        printf ("Error: no device found (%s)\n", dhdErrorGetLastStr ());
        return false;
    }

    m_device_id = dhdOpenType(type.getTypeForSDK());

    if(m_device_id < 0){
        printf ("Error: could not open device (%s)\n", dhdErrorGetLastStr ());
        return false;
    }

    if(drdOpenID(m_device_id) < 0){
        printf ("Error: could not open device in the robotic sdk: (%s)\n", dhdErrorGetLastStr ());
        return false;
    }

    if((!drdIsInitialized()) && (drdAutoInit() < 0)){
        printf("Error: auto-initialization failed: (%s)\n", dhdErrorGetLastStr ());
        return false;
    }

    if(drdStart()){
      printf("Error: starting drd failed: (%s)\n", dhdErrorGetLastStr ());
      return false;
    }

    const char* name = dhdGetSystemName(m_device_id);

    if(name == NULL){
        printf ("Error: no device name found (%s)\n", dhdErrorGetLastStr ());
        return false;
    }
    m_name = name;



    readDeviceStatus();

    return true;
}

bool FdHapticWrapper::performCalibration(bool check_initialization)
{
  if (!drdIsSupported(m_device_id))
  {
    throw std::runtime_error(
      std::string("Device is not supported by force dimension Robotic SDK, so "
                  "calibration is not possible (error: ") +
      getLastError() + ").");
  }

  if (drdAutoInit(m_device_id) < 0)
  {
    drdStop(false, m_device_id);
    throw std::runtime_error(std::string("Auto-initialization failed (error: ") + getLastError() +
                             ")");
  }

  bool initialization_success = true;
  if (check_initialization)
  {
    if (drdCheckInit(m_device_id) < 0)
    {
      initialization_success = false;
    }
  }
  if (drdStop(true, m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not stop Robotic SDK regulation thread (error: ") +
                             getLastError() + ")");
  }

  return initialization_success;
}

FdControllerVersion FdHapticWrapper::readControllerVersion()
{
  FdControllerVersion version;

  if (dhdGetVersion(&version.version, m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not read controller version number (error: ") +
                             getLastError() + "). Shutting down.");
  }

  return version;
}

FdSDKVersion FdHapticWrapper::readSDKVersion()
{
  FdSDKVersion version;
  dhdGetSDKVersion(&version.major, &version.minor, &version.release, &version.revision);
  return version;
}

int FdHapticWrapper::getDeviceId() const
{
  return m_device_id;
}

const std::string& FdHapticWrapper::getName() const
{
  return m_name;
}

std::string FdHapticWrapper::getDeviceStatus() const
{
    return m_status.getStatusText();
}

std::string FdHapticWrapper::getLastError()
{
  return dhdErrorGetLastStr();
}

void FdHapticWrapper::readDeviceStatus()
{
  if (dhdGetStatus(m_status.statusArray(), m_device_id) < 0)
  {
    throw std::runtime_error(std::string("Could not read device status (error: ") + getLastError() +
                             ")");
  }
}

ForceArray::ForceArray()
{
    cartesian_forces_.fill(0.0);
}

ForceArray::ForceArray(std::array<double, 3> cart_forces):
    cartesian_forces_(cart_forces)
{

}

double *ForceArray::atIndex(size_t index)
{
    return &cartesian_forces_.at(index);
}

double *ForceArray::forceX()
{
    return &cartesian_forces_.at(0);
}

double *ForceArray::forceY()
{
    return &cartesian_forces_.at(1);
}

double *ForceArray::forceZ()
{
    return &cartesian_forces_.at(2);
}

PositionArray::PositionArray()
{
    cartesian_position_.fill(0);
}

PositionArray::PositionArray(std::array<double, 3> cart_position):
    cartesian_position_(cart_position)
{

}

double *PositionArray::atIndex(size_t index)
{
    return &cartesian_position_.at(index);
}

double *PositionArray::positionX()
{
    return &cartesian_position_.at(0);
}

double *PositionArray::positionY()
{
    return &cartesian_position_.at(1);
}

double *PositionArray::positionZ()
{
    return &cartesian_position_.at(2);
}

} // namespace fd_haptic_hw


#endif // HAPTIC_WRAPPER_CPP
