#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class colorDetector
{
    public:
        colorDetector();
        void image_cb(const sensor_msgs::CompressedImageConstPtr& msg);

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        ros::Subscriber image_sub_;
        ros::Publisher color_pub_, centroid_x_pub_;
        int low_h_, high_h_, low_s_, high_s_, low_v_, high_v_, blur_k_size_, dil_k_size_;
        int G_low_hsv, G_high_hsv, B_low_hsv, B_high_hsv, R_low_hsv, R_high_hsv;
        int color_choice, flag, color_image, G_mask, B_mask, R_mask, mask;
        int min_pixel;
};

#endif