

#ifndef HAPTIC_WRAPPER_H
#define HAPTIC_WRAPPER_H

#include <array>
#include <string>
#include "drdc.h"
namespace fd_haptic_hw_test {

class ForceArray
{
public:

    ForceArray();

    ForceArray(std::array<double, 3> cart_forces);

    double* atIndex(size_t index);

    double* forceX();
    double* forceY();
    double* forceZ();


private:
    std::array<double, 3> cartesian_forces_;
};

class PositionArray
{
public:

    PositionArray();

    PositionArray(std::array<double, 3> cart_position);

    double* atIndex(size_t index);

    double* positionX();
    double* positionY();
    double* positionZ();


private:
    std::array<double, 3> cartesian_position_;
};

/**
 * \brief Mapping from joints to continuous joint array
 *
 * Encapsulates the mapping from specific joints (as used in the Haptic SDK) to
 * the continuous joint arrays used by ros_control. Without this, the code would
 * be sprinkeled with magic joint indices into joint arrays.
 */
class JointArray
{
public:
  JointArray();

  double* jointValues();
  double* atIndex(size_t i);

  double* jointX();
  double* jointY();
  double* jointZ();
  double* jointEx();
  double* jointEy();
  double* jointEz();
  double* jointGripper();

private:
  std::array<double, 7> m_joint_values;
};

/**
 * \brief Representation of device status
 *
 * This is a simple encapsulation of the device status as described in the
 * Haptic SDK Documentation section Device Status.
 * */
class FdStatus
{
public:
  FdStatus();

  int* statusArray();

  bool power() const;
  bool connected() const;
  bool started() const;
  bool reset() const;
  bool idle() const;
  bool force() const;
  bool brake() const;
  bool torque() const;
  bool wrist_detected() const;
  bool error() const;
  bool gravity() const;
  bool timeguard() const;
  bool wrist_init() const;
  bool redundancy() const;
  bool forceoffcause() const;

  /**
   * Human readable device status description
   *
   * \returns Human readable device status description
   */
  std::string getStatusText() const;

private:
  std::array<int, 16> m_status;
};

/**
 * Force dimension controller version information.
 */
struct FdControllerVersion
{
  double version;
};

/**
 * Force dimension Haptic SDK version information.
 */
struct FdSDKVersion
{
  int major;
  int minor;
  int release;
  int revision;
};

/**
 * Representation of a supported device model
 */
enum class FdDeviceModel
{
  OMEGA_7,
  SIGMA_7
};

/**
 * Representation of the side a device is intended for
 */
enum class FdDeviceSide
{
  LEFT,
  RIGHT
};

/**
 * Representation of a supported device type. Abstracts from DHD_DEVICE_<X>
 * types.
 */
class FdDeviceType
{
public:
  FdDeviceType(FdDeviceModel model, FdDeviceSide side);

  static FdDeviceType fromString(const std::string& model, const std::string& side);

  FdDeviceModel getModel() const;
  FdDeviceSide getSide() const;

  std::string getDescription() const;

  int getTypeForSDK() const;

private:
  FdDeviceModel m_model;
  FdDeviceSide m_side;
};

class FdHapticWrapper
{
public:

  FdHapticWrapper(FdDeviceType& type);

  ~FdHapticWrapper();

  void enable();
  void read(JointArray& pos_target, JointArray& vel_target, JointArray& eff_target);
  void write(JointArray& eff_cmd);
  void moveToPosition(JointArray& pos_target);
  void reset();
  void setMaximumForce(double force);
  void enableForce();
  void disableForce();
  void sleepSeconds(double seconds);
  void getCurrentForce(ForceArray& force);
  void hold();
  void lock(unsigned char mask, bool init);

  bool moveToPosition(PositionArray& pos_target);
  bool forceActive();
  bool requiresCalibration() const;
  bool performCalibration(bool check_initialization);
  bool setCartesianForce(ForceArray& force);
  bool getPositionMoveParameter(PositionArray& pos_array);
  bool isInitialized()const;
  bool setPositionMovementParameters(double a_max, double v_max, double jerk);

  double getMaximumForce();


  int getDeviceId() const;

  const std::string& getName() const;
  std::string getDeviceStatus() const;

  PositionArray getPosition();
  ForceArray getForce();

  FdControllerVersion readControllerVersion();
  FdSDKVersion readSDKVersion();


private:
  int m_device_id;
  std::string m_name;
  FdStatus m_status;
  bool force_activated;
  bool dev_init;

  bool init_device(FdDeviceType& type);

  std::string getLastError();
  void readDeviceStatus();
};

} // namespace fd_haptic_hw
#endif // HAPTIC_WRAPPER_H
