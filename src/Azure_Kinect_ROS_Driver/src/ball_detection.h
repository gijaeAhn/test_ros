//
// Created by sj on 23. 12. 6.
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

namespace ball_detection {


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

    //BUFFER

    int max_history = 20;
    std::vector<k4a::image> k4a_img_buffer(max_history);
    std::deque<cv::Mat> img_buffer;

    // Global
    cv::Rect prev_ball_ROI;
    int candidate_debug = DEBUG_NONE; // For coloring the debug image on similar guess-candidates
    sensor_msgs::CameraInfoPtr info_msg_;

// Constant
    double fx, fy, cx, cy, base_line;
    cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub_, pBackSub_left_, pBackSub_right_;
    std::vector<double> covariance;

// Publisher / Subsciber
    ros::Publisher pose_pub_;
    image_transport::CameraPublisher debug_pub_;

///////////////////////////////////////////////////////////////////////



    cv::Mat k2cvMat(const k4a_image_t& input);

    ros::Time kTime2Ros(uint64_t time);

    void fillCamInfo(k4a_calibration_t);

    double getDepth(double left, double right);

    geometry_msgs::Point getCartesianCoord(double x, double y);

    geometry_msgs::PoseWithCovarianceStamped createEstimateMsg(const geometry_msgs::Point& position, const ros::Time& time_taken);
    void singalHandler(int signum);

    void image_buffer_bookkeeping(const cv::Mat& frame);

    roi_list findMovingCandidates(const cv::Mat& frame);

    void searchCandidates(const cv::Mat& frame, roi_list& ROIs, cv::Rect ball_ROI);

    void findCandidateCenter(const cv::Mat& frame, cv::Rect ball_ROI, cv::Point2f center);

    void publishCenter(point_pair& centers, ros::Time& time_taken);

    void publishDebug(const cv::Mat& frame,roi_list& ROIs, roi_pair& ball_ROIs, point_pair& centers, ros::Time time);

    void detectBall(int image_idx);
}



#endif //TEST_ROS_BALL_PARAMS_H
