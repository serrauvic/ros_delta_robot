#ifndef DELTA_ANGLES_STREAMER_H
#define DELTA_ANGLES_STREAMER_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

class DeltaAnglesStreamer
{
public:
  explicit DeltaAnglesStreamer(const double& steps);
  virtual ~DeltaAnglesStreamer();

  void setCurrentTrajectory(const double &x, const double &y, double theta_values[]);

private:
  /* Variables per calcular els vectors dels desplaçaments */
  /*
  cv::Mat currentPosition_;
  cv::Mat directionVector_;
  cv::Mat uDirectionVector_;
  */
  double steps_;
  cv::Mat currentPosition_ 	= 	(cv::Mat_<double>(2,1) << 0.0, 0.0) ;
  cv::Mat directionVector_	= 	(cv::Mat_<double>(2,1) << 0.0, 0.0) ;
  cv::Mat uDirectionVector_	= 	(cv::Mat_<double>(2,1) << 0.0, 0.0) ;
  //ros node handle
  ros::NodeHandle nh_;
  ros::Publisher angles_pub_;
};

#endif
