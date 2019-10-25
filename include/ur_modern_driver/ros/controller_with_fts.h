#ifndef UR_CONTROLLER_WITH_FTS_
#define UR_CONTROLLER_WITH_FTS_
#include "ur_modern_driver/ros/controller.h"
#include "ur_modern_driver/ros/hardware_interface.h"
#include <force_controllers/force_controller.h>
#include <force_torque_sensor/force_torque_sensor_handle.h>
#include <boost/shared_ptr.hpp>


class ROSController_with_fts : public ROSController
{
  ForceTorqueInterface ftc_interface_;
  // PositionInterface position_interface_with_fts_;
  std::string fts_name;
  std::string fts_sensor_frame;
  std::string fts_transform_frame;
  static const std::string EXTERN_FTS_INTERFACE_NAME;
  boost::shared_ptr<hardware_interface::ForceTorqueSensorHandle> ftsh_;

public:
  ROSController_with_fts(URCommander& commander, TrajectoryFollower& follower, std::vector<std::string>& joint_names,
                        double max_vel_change, std::string tcp_link);
  virtual ~ROSController_with_fts()
  {
  }
  
};

#endif
