#include "delta_robot_drivers/delta_angles_streamer.h"
#include "delta_robot_kinematics/delta_kinematics.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>


DeltaAnglesStreamer::DeltaAnglesStreamer(const double& steps)
  : steps_(steps)
/*:
    currentPosition_(cv::Mat<double>(2,1) << 0.0, 0.0)
  , directionVector_(cv::Mat<double>(2,1) << 0.0, 0.0)
  , uDirectionVector_(cv::Mat<double>(2,1) << 0.0, 0.0)
  */
{
}
DeltaAnglesStreamer::~DeltaAnglesStreamer()
{
}
void DeltaAnglesStreamer::setCurrentTrajectory(const double &x, const double &y, double theta_values[])
{
    //double theta_values [3] = {0, 0, 0};
    double z = -0.35;

    /* Aquí s'entra en matèria, es crea el vector "posició final" en base al topic subscrit */
    cv::Mat goalPosition = (cv::Mat_<double>(2,1) << x, y);
    /* El vector director serà, per tant, l'extrem menys la posició actual*/
    directionVector_ = goalPosition - currentPosition_;
    /* Aquest vector director no es pot aplicar a saco, cal fer-ho en trossos petits cada cert temps.
    Les següents línies miren si el vector director és prou petit. Si ho és, ja l'aplica tot. Per
    comprovar-ho, mira la seva distància i li aplica el valor absolut*/
    if (fabs(cv::norm(directionVector_, cv::NORM_L2)) <  steps_)
    {
      uDirectionVector_ = cv::Mat_<double>::zeros(2, 1);
      currentPosition_  = goalPosition;
    }
    /* Però la majoria de vegades caldrà anar partint la trajectòria. Per partir-la, es calcula
    primer el vector unitari que es correspon a la direcció del vector director. El vector unitari,
    és el mateix vector en direcció i sentit, però el seu mòdul val 1. Quan es multiplica les components
    d'aquest vector per un valor, el mòdul passa a ser aquest valor.*/
    else
    {
      //Calcula el vector unitari
      uDirectionVector_ = directionVector_ / cv::norm(directionVector_, cv::NORM_L2);
      //Multiplica el vector unitari per un valor petit i el suma a la posició actual
      currentPosition_ = currentPosition_ + uDirectionVector_ * steps_;
    }
    // IK
    delta_kinematics::inversekinematics(
      currentPosition_.at<double>(0,0), currentPosition_.at<double>(1,0), z, theta_values);

    // Rad to degrees
    theta_values[0] = conversio * theta_values[0];
    theta_values[1] = conversio * theta_values[1];
    theta_values[2] = conversio * theta_values[2];
}
