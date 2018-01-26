// SYSTEM
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
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

#include "std_msgs/UInt16MultiArray.h"


namespace fan_hwiface
{
	// For simulation only - determines how fast a trajectory is followed
	static const double POSITION_STEP_FACTOR = 10;

class DeltaHWDriver : public hardware_interface::RobotHW
{
public:

	DeltaHWDriver() {}
	virtual ~DeltaHWDriver() {}

	std::string robot_namespace_;

	std::string urdf_string_;
	urdf::Model urdf_model_;


	realtime_tools::RealtimePublisher<std_msgs::UInt16MultiArray> *pub_commands_;
	ros::Subscriber sub_measures_;

	ros::Subscriber angles_sub_;
	ros::Subscriber arduino_feedback_angles_sub_;

	double thetas_[3] = {0, 0, 0};
	double feed_back_thetas_[3] = {0, 0, 0};

	bool init(ros::NodeHandle &nh, std::string robot_name)
	{
		n_joints_ = 3;

		loop_hz_ = 0.1;

		pub_commands_ = new realtime_tools::RealtimePublisher<std_msgs::UInt16MultiArray>(nh, "arduino_cmd", 1);
		//sub_measures_ = nh.subscribe("potentiometer_msr", 1000, &DeltaHWDriver::updateMsr, this) ;

		angles_sub_ = nh.subscribe<std_msgs::UInt16MultiArray>("/delta_robot_drivers/trajectory_angles", 1000, &DeltaHWDriver::updateAngles, this) ;

		arduino_feedback_angles_sub_= nh.subscribe<std_msgs::UInt16MultiArray>("/arduino/angles", 1000, &DeltaHWDriver::updateArduinoAngles, this) ;


		nh.getParam("gain", gain_);
		nh.getParam("bias", bias_);

		robot_namespace_ = robot_name;
		joint_names_.push_back( std::string("link_0_JOINT_1") );
		joint_names_.push_back( std::string("link_0_JOINT_2") );
		joint_names_.push_back( std::string("link_0_JOINT_3") );

		joint_position_.resize(n_joints_);
		joint_position_prev_.resize(n_joints_);
		joint_velocity_.resize(n_joints_);
		joint_velocity_command_.resize(n_joints_);

		joint_lower_limits_.resize(n_joints_);
		joint_upper_limits_.resize(n_joints_);

		joint_position_command_.resize(n_joints_);

		for (int j = 0; j < n_joints_; ++j)
		{
			joint_position_[j] = 0.0;
			joint_position_prev_[j] = 0.0;
			joint_velocity_[j] = 0.0;
			joint_velocity_command_[j] = 0.0;
			joint_position_command_[j] = 0.0;
		}

		for(int j=0; j < n_joints_; j++)
		{
			state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_velocity_[j]));

			// Create position joint interface
      position_joint_interface_.registerHandle(hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),&joint_position_command_[j]));


			//hardware_interface::JointHandle joint_handle_velocity;
			//joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]), &joint_velocity_command_[j]);
			//velocity_interface_.registerHandle(joint_handle_velocity);
		}

		registerInterface(&state_interface_);
		registerInterface(&position_joint_interface_);
		//registerInterface(&velocity_interface_);

		return true;
	};

	void read(ros::Time time, ros::Duration period)
	{
		//ROS_INFO("Read from arduino");

		joint_position_prev_[0] = joint_position_[0];
		joint_position_command_[0] = joint_position_[0];
		joint_position_[0] = thetas_[0];// -10;

		joint_position_prev_[1] = joint_position_[1];
		joint_position_command_[1] = joint_position_[1];
		joint_position_[1] = thetas_[1];// -11;

		joint_position_prev_[2] = joint_position_[2];
		joint_position_command_[2] = joint_position_[2];
		joint_position_[2] = thetas_[2];// -12;
/*
		joint_position_command_[0] = thetas_[0];
		joint_position_command_[1] = thetas_[1];
		joint_position_command_[2] = thetas_[2];
*/
		return;
	};

	void write(ros::Time time, ros::Duration period)
	{
		//ROS_INFO("Write to arduino");
		if (pub_commands_->trylock())
		{
			std_msgs::UInt16MultiArray angles;
      angles.layout.dim.push_back(std_msgs::MultiArrayDimension());
      angles.layout.dim[0].size = 3;
      angles.layout.dim[0].label = "thetas";
      angles.data.clear();

			pub_commands_->msg_.data.clear();
			// Move all the states to the commanded set points slowly
		  for (std::size_t i = 0; i < n_joints_; ++i)
			{
				//ROS_INFO("Joint joint_position_command_ %d = %f", i, joint_position_command_[i]);
				//ROS_INFO("Joint joint_position_ %d = %f", i, joint_position_[i]);
				// Position
				p_error_ = joint_position_command_[i] - joint_position_[i];
				// scale the rate it takes to achieve position by a factor that is invariant to the feedback loop
        joint_position_[i] += p_error_ * POSITION_STEP_FACTOR / loop_hz_;

				//ROS_INFO("Joint joint_position_command_ %d = %f", i, joint_position_command_[i]);
				//ROS_INFO("Joint joint_position_ %d = %f", i, joint_position_[i]);
				//ROS_INFO("Joint joint_position_ ERROR %d = %f", i, p_error_);

			}
			//ROS_INFO("Joint joint_position_ %d = %f", 0, joint_position_command_[0]);
			angles.data.push_back(joint_position_command_[0]);
			angles.data.push_back(joint_position_command_[1]);
			angles.data.push_back(joint_position_command_[2]);
			pub_commands_->msg_ = angles;

/*
			// only actuate in the correct direction
			double fan_speed = ( ( joint_velocity_command_.at(1) ) );
			if( fan_speed > 0.0 )
				fan_speed = 0;
			fan_speed = std::abs(fan_speed);
			pub_commands_->msg_.data = static_cast<uint8_t>(1023*fan_speed/5);
			if( pub_commands_->msg_.data > 255 )
				pub_commands_->msg_.data = 255;
*/
			//ROS_INFO("pub_command %f", pub_commands_->msg_.data);

			pub_commands_->unlockAndPublish();
		}
		return;
	};

	// hardware interfaces
	hardware_interface::JointStateInterface state_interface_;
	hardware_interface::VelocityJointInterface velocity_interface_;
	hardware_interface::PositionJointInterface position_joint_interface_;


	// joint limits interfaces
	joint_limits_interface::VelocityJointSaturationInterface   vj_sat_interface_;
	joint_limits_interface::VelocityJointSoftLimitsInterface   vj_limits_interface_;
	joint_limits_interface::PositionJointSaturationInterface   pj_sat_interface_;
	joint_limits_interface::PositionJointSoftLimitsInterface   pj_limits_interface_;

	// Before write, you can use this function to enforce limits for all values
	void enforceLimits(ros::Duration period);

	// configuration
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
	joint_velocity_command_,
	joint_position_command_;

	// aux var to update the angle
	double angle_;

	// sensor calibration
	double gain_;
	double bias_;

	// set all members to default values
	void reset();

	// transmissions in this plugin's scope
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

 double p_error_, v_error_, e_error_;
 double loop_hz_;

private:

	void updateMsr(const std_msgs::UInt16::ConstPtr& msg)
	{
		angle_ = gain_* msg->data + bias_;
	}
	void updateAngles(const std_msgs::UInt16MultiArray::ConstPtr&  angles_msg_ptr)
	{
		std_msgs::UInt16MultiArray angles_msg = *angles_msg_ptr;

		ROS_INFO("Received new angles to send: %d, %d, %d", angles_msg.data[0], angles_msg.data[1], angles_msg.data[2]);

		thetas_[0] = angles_msg.data[0];
		thetas_[1] = angles_msg.data[1];
		thetas_[2] = angles_msg.data[2];


	}
	void updateArduinoAngles(const std_msgs::UInt16MultiArray::ConstPtr&  angles_msg_ptr)
	{
		std_msgs::UInt16MultiArray angles_msg = *angles_msg_ptr;

		ROS_INFO("Received Arduino angles: %d, %d, %d", angles_msg.data[0], angles_msg.data[1], angles_msg.data[2]);

			feed_back_thetas_[0] = angles_msg.data[0];
			feed_back_thetas_[1] = angles_msg.data[1];
			feed_back_thetas_[2] = angles_msg.data[2];
	}


}; // class

} // namespace

bool g_quit = false;

void quitRequested(int sig)
{
	g_quit = true;
}

int main( int argc, char** argv )
{
	// initialize ROS
	ros::init(argc, argv, "delta_robot_driver_node", ros::init_options::NoSigintHandler);

	// ros spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// custom signal handlers
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	// create a node
	ros::NodeHandle copter_arm_nh;

/*
	ros::NodeHandle nh;

	ros::Subscriber angles =
      nh.subscribe<geometry_msgs::Vector3>("/delta_img_processor/center_ray_direction", 1,
                      boost::bind(setCurrentTrajectory, _1, boost::ref(streamer)));
*/

	// get params or give default values
	std::string name;
	copter_arm_nh.param("name", name, std::string("delta_robot"));

	// advertise the e-stop topic
	// ros::Subscriber estop_sub = copter_arm_nh.subscribe(copter_arm_nh.resolveName("emergency_stop"), 1, eStopCB);

	// get the general robot description, the lwr class will take care of parsing what's useful to itself
	// std::string urdf_string = getURDF(copter_arm_nh, "/robot_description");

	// construct and start the real lwr
	fan_hwiface::DeltaHWDriver copter_arm_hwiface;
	// copter_arm_hwiface.create(name, urdf_string);
	if(!copter_arm_hwiface.init(copter_arm_nh, "delta_robot"/*name*/))
	{
		ROS_FATAL_NAMED("lwr_hw","Could not initialize robot real interface");
		return -1;
	}

	// timer variables
	struct timespec ts = {0, 0};
	ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
	ros::Duration period(1.0);

	//the controller manager
	controller_manager::ControllerManager manager(&copter_arm_hwiface, copter_arm_nh);

	ros::Rate rate(100);
	while( !g_quit )
	{
		// get the time / period
		if (!clock_gettime(CLOCK_REALTIME, &ts))
		{
			now.sec = ts.tv_sec;
			now.nsec = ts.tv_nsec;
			period = now - last;
			last = now;
		}
		else
		{
			ROS_FATAL("Failed to poll realtime clock!");
			break;
		}

		// read the state from the lwr
		copter_arm_hwiface.read(ros::Time::now(), period);

		// Compute the controller commands
		bool resetControllers = true;
		/*if(!wasStopHandled && !resetControllers)
		{
			ROS_WARN("E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
			ROS_WARN("HOW TO RELEASE E-STOP: rostopic pub -r 10 /NAMESPACE/emergency_stop std_msgs/Bool 'data: false'");
			resetControllers = true;
			wasStopHandled = true;
		}

		if( isStopPressed )
		{
			wasStopHandled = false;
		}
		else
		{
			resetControllers = false;
			wasStopHandled = true;
		}  */

		// update the controllers
		manager.update(ros::Time::now(), period, resetControllers);

		// write the command to the lwr
		copter_arm_hwiface.write(ros::Time::now(), period);

		rate.sleep();
	}

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;
}
