#include "delta_robot_img_processor/ros_img_processor.h"
#include "delta_robot_img_processor/circle_detector.h"
#include "delta_robot_img_processor/camera.h"

//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
	//loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);
  ray_direction_circle_pub   = nh_.advertise<geometry_msgs::Vector3>("center_ray_direction", 1);

  ray_direction_ = (cv::Mat_<double>(3,1) << 0, 0, 0) ;

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    cv::Rect_<int> box;

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;
        // detected circles-
        std::vector<cv::Vec3f> circles;

        // detect circles in the image.
        Hough_Transform::calculate(cv_img_out_.image, circles);

        //draw circles on the image
        for(unsigned int ii = 0; ii < circles.size(); ii++ )
        {
            // if valid circle.
            if ( circles[ii][0] != -1 )
            {
                // center of the circle.
                cv::Point center;
                // radius of the circle.
                int radius;

                // get cirlce coodinates, center point and radius.
                Hough_Circle::get_center_coordinates(circles[ii], center, radius);
                // draw circle.
                draw_clircle(center, radius, true/*draw circle center coordinates*/);
                // calculate center circle ray direction from camera frame persepctive,
                // put the circle center point in the real world.
                Camera::get_ray_direction(matrixK_, center, ray_direction_);
                // draw vector.
                //draw_ray_direction_vector(center);
            }
        }

        //sets and draw a bounding box around the ball
        //box.x = (cv_img_ptr_in_->image.cols/2)-10;
        //box.y = (cv_img_ptr_in_->image.rows/2)-10;
        //box.width = 20;
        //box.height = 20;
        //cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}
void RosImgProcessorNode::draw_clircle(const cv::Point & center, int radius, bool draw_center_coordinates)
{
  // circle center in yellow
  cv::circle(cv_img_out_.image, center, 5, cv::Scalar(255, 255, 0), -1, 8, 0 );
  // circle perimeter in purple.
  cv::circle(cv_img_out_.image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_8, 0 );

  if (draw_center_coordinates)
  {
    // circle center point x and y coordinates.
    std::ostringstream stringStream;
    stringStream  << "  x:" << center.x << "\n" << " y:" << center.y;
    // print circle center coordinates
    cv::putText(cv_img_out_.image, stringStream.str(), center, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 153, 51), 2, 0.5);
  }
}
void RosImgProcessorNode::draw_ray_direction_vector(const cv::Point & center)
{
  // line from center circle
  cv::line( cv_img_out_.image, center, cv::Point( ray_direction_.at<double>(0, 0), ray_direction_.at<double>(1, 0) ), cv::Scalar( 110, 220, 0 ),  2, 8 );
}
void RosImgProcessorNode::publish()
{
    //image_raw topic
	if(cv_img_out_.image.data)
	{
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());

      // publish center ray direction.
      geometry_msgs::Vector3 direction;
      direction.x = ray_direction_.at<double>(0, 0); //Minus added for matching the robot mounting bracket
      direction.y = -ray_direction_.at<double>(1, 0);
      direction.z = ray_direction_.at<double>(2, 0);
      ray_direction_circle_pub.publish(direction);
	}
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);
	//std::cout << matrixP_ << std::endl;

  matrixK_ = (cv::Mat_<double>(3,3) << _msg.K[0],_msg.K[1],_msg.K[2],
                                        _msg.K[3],_msg.K[4],_msg.K[5],
                                        _msg.K[6],_msg.K[7],_msg.K[8]);
//std::cout << matrixK_ << std::endl;
}
