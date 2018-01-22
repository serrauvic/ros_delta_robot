#include "delta_robot_drivers/delta_angles_streamer.h"

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "geometry_msgs/Vector3.h"

void setCurrentTrajectory(const geometry_msgs::Vector3::ConstPtr& circle_center,
                          DeltaAnglesStreamer& streamer)
{

    //ROS_INFO("Setting new trajectory x: %d y:%d", circle_center->x, circle_center->y);
    geometry_msgs::Vector3 new_circle_center = *circle_center;
    ROS_INFO("=========================");
    ROS_INFO("Setting new trajectory x: %f y: %f", new_circle_center.x, new_circle_center.y);

    /*Pot passar que si el node de la càmera no existeix, el valor subscrit valdrà
        un Not A Number. Això és un problema si es fa càlcul numèric. Les següents linies
        eviten que això passi i li donen un valor conegut.
    */
    double theta_values [3] = {0, 0, 0};

    streamer.setCurrentTrajectory(new_circle_center.x, new_circle_center.y, theta_values);

    ROS_INFO("Trajectory angles Theta1: %f Theta2: %f Theta3: %f"
    , theta_values[0], theta_values[1], theta_values[2]);

    ROS_INFO("=========================");
}

/* Els següents defines haurien d'acabar sent paràmetres al ROS launch */
#define DEF_MAX_STEP_SIZE 0.1
#define DE_EXECUTION_FREQUENCY 2 /* Hz */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "delta_angles_streamer_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double max_steps, exec_freq;
  std::string ip_addr;
  pnh.param<double>("max_steps", max_steps, DEF_MAX_STEP_SIZE);
  pnh.param<double>("exec_freq", exec_freq, DE_EXECUTION_FREQUENCY);

  ros::AsyncSpinner spinner(1);

  spinner.start();

  ros::Rate loop_rate(exec_freq);

  DeltaAnglesStreamer streamer(max_steps);

  ros::Subscriber coord =
      nh.subscribe<geometry_msgs::Vector3>("/delta_img_processor/center_ray_direction", 1,
                      boost::bind(setCurrentTrajectory, _1, boost::ref(streamer)));

  while (ros::ok())
  {
    //execute pending callbacks
    //ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::waitForShutdown();
  //ros::spin();
  return 0;
}
