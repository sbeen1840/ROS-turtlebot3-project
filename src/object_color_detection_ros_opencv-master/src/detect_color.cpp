#include <cv_bridge/cv_bridge.h>
#include "detect_color.h"
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>

colorDetector::colorDetector()
    : it_(nh_)
{
    // subscribe
    image_sub_ = nh_.subscribe("/raspicam_node/image/compressed", 1, &colorDetector::image_cb, this);
    //if in simulation mode  ("/camera/rgb/image_raw");

    // publish
    color_pub_ = nh_.advertise<std_msgs::Int8>("/color", 1);
    centroid_x_pub_ = nh_.advertise<std_msgs::Int16>("/circle", 1);


}

void colorDetector::image_cb(const sensor_msgs::CompressedImageConstPtr& msg)
{
    // variables
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr before_ptr;
    std_msgs::Int8 color_msg;

    min_pixel = 500;
    blur_k_size_ = 1;
    dil_k_size_ = 1;


    //try or catch
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        before_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::cout<<"이미지 입력 됨"<<std::endl;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        std::cout<<"이미지 입력 안됨"<<std::endl;
        return;
    }


    // image processing
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
    cv::Mat G_mask, B_mask, R_mask, mask;

    cv::inRange(hsv_image, cv::Scalar(40,60,60), cv::Scalar(90, 255, 255), G_mask); //초록색 검출
    cv::inRange(hsv_image, cv::Scalar(90,113,70), cv::Scalar(130, 255, 255), B_mask); //파란색 검출
    cv::inRange(hsv_image, cv::Scalar(150,170,70), cv::Scalar(179,255,155), R_mask); //빨간색 검출

    cv::blur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_));
    cv::dilate(cv_ptr->image, cv_ptr->image, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * dil_k_size_ + 1, 2 * dil_k_size_ + 1), cv::Point(dil_k_size_, dil_k_size_)));
    cv::erode(cv_ptr->image, cv_ptr->image, 5);

    cv::bitwise_or(G_mask, B_mask, mask);


// ! color detection for motor & manipulator action ---------------------
// * 파란색 혹은 초록색 특정 픽셀 이상이면
    if (countNonZero(mask) > min_pixel)
    {
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 3, mask.rows / 16, 200, 80, 0, 0);

        std_msgs::Int16 centroid_x_msg;
        int green_pixel = cv::countNonZero(G_mask);
        int blue_pixel = cv::countNonZero(B_mask);
        int radius =0;

        // * 초록색 특정 픽셀 이상이면
        if (green_pixel > min_pixel){
            std::cout <<"초록색 \n 픽셀수 : "<< green_pixel << std::endl;

            if (!circles.empty()){
                for (const auto& circle : circles) {
                    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                    radius = cvRound(circle[2]);

                    if (radius > 0.001){
                        cv::circle(before_ptr->image, center, radius, cv::Scalar(0, 255, 0), 2);
                        cv::circle(before_ptr->image, center, 3, cv::Scalar(0, 0, 255), -1);
                        std::cout << "초록색 원의 중심: (" << center.x << ", " << center.y << ")" << std::endl;
                        centroid_x_msg.data = center.x;
                        centroid_x_pub_.publish(centroid_x_msg);

                        color_msg.data = 1;
                        std::cout <<"초록색 원 발견됐으니 센터트래킹 시작" << std::endl;

                        if (green_pixel >= 7800){
                            color_msg.data = 4;
                            std::cout <<"초록색 원이 가까워졌으니 동작1 시작" << std::endl;
                        }
                    }
                }
            }
            else{
            std::cout <<"초록색이나 원은 발견 못함" << std::endl;
            }

        }
        // * 파란색 특정 픽셀 이상이면
        else if (blue_pixel > min_pixel){
            std::cout <<"파란색 \n 픽셀수 : "<< blue_pixel << std::endl;

            if (!circles.empty()){
                for (const auto& circle : circles) {
                    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                    radius = cvRound(circle[2]);

                    if (radius > 0.001){
                        cv::circle(before_ptr->image, center, radius, cv::Scalar(0, 255, 0), 2);
                        cv::circle(before_ptr->image, center, 3, cv::Scalar(0, 0, 255), -1);
                        std::cout << "파란색 원의 중심: (" << center.x << ", " << center.y << ")" << std::endl;
                        centroid_x_msg.data = center.x;
                        centroid_x_pub_.publish(centroid_x_msg);

                        color_msg.data = 2;
                        std::cout <<"파란색 원 발견됐으니 센터트래킹 시작" << std::endl;

                        if (blue_pixel >= 10000) {
                            color_msg.data = 5;
                            std::cout <<"파란색 원이 가까워졌으니 동작2 시작" << std::endl;
                        }
                    }
                }
            }
            else{
            std::cout <<"파란색이나 원은 발견 못함" << std::endl;
            }
        }

        else{
            std::cout <<"mask에 문제 있음" << std::endl;
        }
    }


// * 3. 빨간색이 검출된 경우
    else if (cv::countNonZero(R_mask) > 3000)
    {
        color_msg.data = 3;
        std::cout <<"빨간색 발견했으니 정지" << std::endl;
    }

// * 4. 색이 검출되지 않은 경우
    else
    {
        color_msg.data = 0;
        std::cout << "색감지 되지 않으니 자율주행 모드" << std::endl;
    }


    // publish and visualize
    color_pub_.publish(color_msg);
    cv::imshow("Before", before_ptr->image);
    cv::waitKey(1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ColorDetector");
    colorDetector cd;
    ros::spin();
    return 0;
}