#ifndef CAMERA_H
#define CAMERA_H

//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

//TODO: In a future it could be a class for now with an inlined function it's enough.
namespace Camera
{
  // calculate rady direction from Hough detected ciercle using the camera K (Intrinsic calibration matrix)
  // d = K^-1 * u.
  // where u is the center of the circle.
  void get_ray_direction(const cv::Mat &matrixK, const cv::Point &center, cv::Mat &ray_direction)
  {
    // Make room for the inverted K matrix.
    cv::Mat kinverted;
    // Inver the K matrix.
    cv::invert(matrixK, kinverted, cv::DECOMP_LU);
    // Make room and initialize ray direction value.
    ray_direction = (cv::Mat_<double>(3,1) << 0, 0, 0);
    // Homogenous point (give it a third coordinate equal to 1).
    cv::Mat homogenous_point = (cv::Mat_<double>(3,1) << center.x, center.y, 1.0);
    // Put in world coordinates the circle center.
    ray_direction = kinverted * homogenous_point;

  };
} // Camera
#endif
