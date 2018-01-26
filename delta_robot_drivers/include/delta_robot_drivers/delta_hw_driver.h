#ifndef DELTA_HW_DRIVER_H
#define DELTA_HW_DRIVER_H

// SYSTEM
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/filters.h>
/*
class DeltaHWDriver : public hardware_interface::RobotHW
{
public:
  explicit DeltaHWDriver();
  virtual ~DeltaHWDriver();

  bool init(ros::NodeHandle &nh, std::string robot_name);
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

private:

  //void updateMsr(const std_msgs::UInt16::ConstPtr& msg);

  int n_joints_;
  std::vector<std::string> joint_names_;

  // limits
	std::vector<double>
	joint_lower_limits_,
	joint_upper_limits_;

	// state and commands
	std::vector<double>
	joint_position_,
	joint_position_prev_,
	joint_velocity_,
	joint_velocity_command_;

  // aux var to update the angle
	double angle_;

	// sensor calibration
	double gain_;
	double bias_;

	// set all members to default values
	void reset();

	// transmissions in this plugin's scope
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

}
*/
#endif
