#include "ur_modern_driver/ros/controller_with_fts.h"

ROSController_with_fts::ROSController_with_fts(URCommander& commander, TrajectoryFollower& follower,
                             std::vector<std::string>& joint_names, double max_vel_change, std::string tcp_link)
  : ROSController(commander, follower, joint_names, max_vel_change, tcp_link)
  , ftc_interface_(follower, jhs_, joint_names)
{
    nh_.param<std::string>("/fts/FTS/name", fts_name, "fts_sensor_name");
    nh_.param<std::string>("/fts/Node/transform_frame", fts_transform_frame, "fts_base_link");
    ROS_DEBUG_STREAM("try to register the force torque controller interface to simulation env.");

    // hardware_interface::RobotHW::registerInterface<"force_torque_control::ForceTorqueInterface">(&ftc_interface_);
    registerControllereInterface(&ftc_interface_);
    
    available_interfaces_["force_torque_control::ForceTorqueInterface"] = &ftc_interface_;
    ROS_DEBUG_STREAM("successfully registered force torque controller interface");
    
    // create a robot handle for kr5, and attach the fake sensor handle
    ROS_DEBUG_STREAM("creating ForceTorqueControllerHandle");
    force_torque_control::ForceTorqueControllerHandle fts_handle("ur_fts_controller_handle");
    
    ROS_DEBUG_STREAM("creating ForceTorqueSensorHandle");
    ftsh_.reset(new force_torque_sensor::ForceTorqueSensorHandle(nh_, fts_name, fts_transform_frame));
    ROS_INFO_STREAM("ForceTorqueSensorHandle created!");
    
    fts_handle.addHandle(*ftsh_);
    ROS_INFO("force torque sensor handle added");
    

    // add joint handle.
    for(size_t i = 0; i < joint_names.size(); i ++)
    {
        fts_handle.addPosHandle(getPosHandle(joint_names[i]));
        ROS_ERROR_STREAM("Controller FTS: added handle" << joint_names[i]);
    }
    ROS_DEBUG_STREAM("add position handle");

    for(size_t i = 0; i < joint_names.size(); i ++)
        fts_handle.addVelHandle(getVelHandle(joint_names[i]));
    ROS_DEBUG_STREAM("add velocity handle");
    
    ftc_interface_.registerHandle(fts_handle);
    ROS_INFO("register handle to ftc interface");    
    
    
}
