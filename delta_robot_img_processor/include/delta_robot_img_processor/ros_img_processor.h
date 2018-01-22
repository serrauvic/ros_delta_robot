#ifndef ROS_IMG_PROCESSOR_H
#define ROS_IMG_PROCESSOR_H

//std C++
#include <iostream>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "geometry_msgs/Vector3.h"

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */
class RosImgProcessorNode
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;

        //image transport
        image_transport::ImageTransport img_tp_;

        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs_;
        ros::Subscriber camera_info_subs_;

        //publishers
        image_transport::Publisher image_pub_;
        ros::Publisher ray_direction_circle_pub;

        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;
        // ray direction in camera frame.
        cv::Mat ray_direction_;

    		//Camera matrix
    		cv::Mat matrixP_;
        cv::Mat matrixK_;

        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;

    protected:
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & _msg);
        void draw_clircle(const cv::Point & center, const int radius, bool draw_center_coordinates);
        void draw_ray_direction_vector(const cv::Point & center);

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        RosImgProcessorNode();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~RosImgProcessorNode();

        /** \brief Process input image
        *
        * Process input image
        *
        **/
        void process();

        /** \brief Publish output image
        *
        * Publish output image
        *
        */
        void publish();

        /** \brief Returns rate_
         *
         * Returns rate_
         *
         **/
        double getRate() const;
};
#endif
