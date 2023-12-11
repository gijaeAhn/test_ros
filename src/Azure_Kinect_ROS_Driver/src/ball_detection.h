//
// Created by gj on 23. 12. 6.
//

#ifndef BALL_PARAMS_H
#define BALL_PARAMS_H

#include <algorithm>
#include <csignal>
#include <thread>
#include <chrono>
#include <deque>
#include <queue>
#include <string>
#include <iostream>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/video/background_segm.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>

#include <k4a/k4a.hpp>

#include <ros/ros.h>

typedef std::pair<cv::Rect ,cv::Rect> roi_pair;
typedef std::pair<cv::Point2f , cv::Point2f> point_pair;
typedef std::vector<cv::Rect> roi_list;
typedef std::vector<cv::Point> Contour;
typedef std::vector<Contour> Contours;


#define DEBUG_GOOD  0 // Green
#define DEBUG_MAYBE 1 // Blue
#define DEBUG_BAD   2 // Red
#define DEBUG_NONE  3 // Purple




    //ROS PARAMS
    std::string camera_id_;
    cv::Scalar low_HSV_, high_HSV_;
    double background_resize_;
    double z_k2, z_k1, z_k0, xy_k2, xy_k1, xy_k0;
    bool print_diagnostics_;
    double guess_max_area_diff_, guess_max_square_diff_, guess_max_dist_diff_;
    double lockin_max_dist_, lockin_wait_;
    bool debug_minimal_;
    double debug_resize;
    double debug_resize_w;
    double debug_resize_h;
    bool largest_moving_center_;
    bool is_merged_;
    double merged_percent_;
    double debug_publish_period_;
    bool resize_frame;

    //BUFFER

    int max_history = 20;
    std::deque<k4a_image_t > k4a_rgb_buffer(max_history);
    std::deque<k4a_image_t > k4a_depth_buffer(max_history);
    std::deque<cv::Mat> cv_buffer;

    // Global
    cv::Rect prev_ball_ROI;
    int candidate_debug = DEBUG_NONE; // For coloring the debug image on similar guess-candidates
    sensor_msgs::CameraInfoPtr info_msg_;

// Constant
    double fx, fy, cx, cy, base_line;
    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub_;
    std::vector<double> covariance;

// Publisher / Subsciber
    ros::Publisher pose_pub_;
    image_transport::CameraPublisher debug_pub_;
    image_transport::CameraPublisher debug_thresh_pub_;

///////////////////////////////////////////////////////////////////////



    cv::Mat k2cvMat(const k4a_image_t& input);

    ros::Time kTime2Ros(const uint64_t & time);

    void fillCamInfo(k4a_calibration_t& cali);

    double getDepth(int x, int y);



    geometry_msgs::Point getCartesianCoord(int x,int y,uint16_t depth);

    geometry_msgs::PoseWithCovarianceStamped createEstimateMsg(const geometry_msgs::Point& position, const ros::Time& time_taken);

    void signalHandler(int signum);

    void cv_buffer_bookkeeping(const cv::Mat& frame);

    void rgb_buffer_bookkeeping(const k4a_image_t& frame);

    void depth_buffer_bookkeeping(const k4a_image_t& frame);

    geometry_msgs::PoseWithCovarianceStamped createEstimateMsg(const geometry_msgs::Point& position , ros::Time& taken_time);


    roi_list findMovingCandidates(const cv::Mat& frame);

    void searchCandidates(const cv::Mat& frame, roi_list& ROIs, cv::Rect& ball_ROI);

    void findCandidateCenter(const cv::Mat& frame, cv::Rect& ball_ROI, cv::Point2i& center);

    void publishCenter(cv::Point2i& center, ros::Time& time_taken, k4a_image_t& depth_image);

    void publishDebug(const cv::Mat& frame,roi_list& ROIs, cv::Rect& ball_ROI, cv::Point2i center, ros::Time time);

    void detectBall();

    k4a_result_t getDepthImage(const k4a::capture& capture, k4a::image& return_depth_image);

    k4a_result_t getRGBImage(const k4a::capture& capture, k4a::image& return_RGB_image);




#endif //TEST_ROS_BALL_PARAMS_H
