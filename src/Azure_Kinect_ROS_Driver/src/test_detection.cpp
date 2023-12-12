//
// Created by gj on 23. 12. 8.
//
#include "ball_detection.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "ball_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    camera_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    camera_config.synchronized_images_only = true;
//    camera_config.subordinate_delay_off_master_usec = 0;     // Must be zero for master
//    camera_config.synchronized_images_only = true;


    uint32_t device_count = k4a_device_get_installed_count();


    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    k4a_device_t device = NULL;
    k4a_capture_t capture = NULL;

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device,&camera_config))
    {
        ROS_INFO_STREAM("FAILED TO START CAMERAS");
        return 0;
    }


    ROS_INFO_STREAM("STARTING CAMERAS");

    nhp.getParam("camera_id", camera_id_);
    nhp.param("low_H", low_HSV_[0], 0.0);
    nhp.param("low_S", low_HSV_[1], 0.0);
    nhp.param("low_V", low_HSV_[2], 200.0);
    nhp.param("high_H", high_HSV_[0], 180.0);
    nhp.param("high_S", high_HSV_[1], 50.0);
    nhp.param("high_V", high_HSV_[2], 255.0);
    nhp.param("print_diagnostics", print_diagnostics_, true);
    nhp.param("z_k2", z_k2, 0.002);
    nhp.param("z_k1", z_k1, 0.001);
    nhp.param("z_k0", z_k0, 0.01);
    nhp.param("xy_k2", xy_k2, 0.0003);
    nhp.param("xy_k1", xy_k1, 0.0004);
    nhp.param("xy_k0", xy_k0, 0.01);
    nhp.param("guess_max_area_diff", guess_max_area_diff_, 1.5);
    nhp.param("guess_max_square_diff", guess_max_square_diff_, 2.0);
    nhp.param("guess_max_dist_diff", guess_max_dist_diff_, -1.0);
    nhp.param("lockin_wait", lockin_wait_, 0.1);
    nhp.param("lockin_max_dist", lockin_max_dist_, 2.0);
    nhp.param("background_resize", background_resize_, 0.4);
    nhp.param("debug_minimal", debug_minimal_, false);
    nhp.param("debug_resize", debug_resize_w, 0.4);
    nhp.param("debug_resize", debug_resize_h, 0.4);
    nhp.param("largest_moving_center", largest_moving_center_,true);
    nhp.param("merged_percent", merged_percent_, 0.05);
    nhp.param("debug_publish_period", debug_publish_period_, 0.01);


    k4a_calibration_t cali;
    k4a_device_get_calibration(device,camera_config.depth_mode,camera_config.color_resolution,&cali);


    auto rgb_cali = cali.color_camera_calibration;

    fx = rgb_cali.intrinsics.parameters.param.fx;
    cx = rgb_cali.intrinsics.parameters.param.cx;
    fy = rgb_cali.intrinsics.parameters.param.fy;
    cy = rgb_cali.intrinsics.parameters.param.cy;
    printf( "fx ,cx ,fy cy %f %f %f %f", fx,cx,fy, cy);


    info_msg_.reset(new sensor_msgs::CameraInfo());
    fillCamInfo(cali);

    pose_pub_ = nhp.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ball_" + camera_id_ + "_pose", 1);

    image_transport::ImageTransport it(nhp);
    debug_pub_ = it.advertiseCamera("debug_img", 1);
    debug_thresh_pub_ = it.advertiseCamera("debug_thresh_img",1);
    ROS_INFO_STREAM("Advertised on topic " << debug_pub_.getTopic());
    ROS_INFO_STREAM("Advertised on topic " << debug_pub_.getInfoTopic());

    pBackSub_ = cv::createBackgroundSubtractorMOG2(100, 30, false);


    covariance = std::vector<double>(36, 0.0);

    ros::Time prev_loop_time = ros::Time::now();

    std::signal(SIGINT, signalHandler); // Free memory so don't get error

    std::chrono::milliseconds i(1000);

    while(ros::ok()){
        //Capture Image and Keeping
        auto detect_start = ros::Time::now();
        k4a_image_t image;
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a capture\n");
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                return 0;
        }

        k4a_image_t RGBIMAGE = k4a_capture_get_color_image(capture);
        k4a_image_t DEPTHIMAGE = k4a_capture_get_depth_image(capture);
        printf("Detph Size : %d %d", k4a_image_get_height_pixels(DEPTHIMAGE), k4a_image_get_width_pixels(DEPTHIMAGE));
        printf("RGB Size : %d %d", k4a_image_get_height_pixels(RGBIMAGE), k4a_image_get_width_pixels(RGBIMAGE));
        cv::Mat RGBmat = k2cvMat(RGBIMAGE);
        rgb_buffer_bookkeeping(RGBIMAGE);
        depth_buffer_bookkeeping(DEPTHIMAGE);
        cv_buffer_bookkeeping(RGBmat);
        detectBall();


        auto detect_end = ros::Time::now();

        if (print_diagnostics_) {
            ros::Time curr_time = ros::Time::now();
            ROS_INFO_STREAM_THROTTLE(1, "--- Diagnostics CPU: " << camera_id_ << " ---");
            ROS_INFO_STREAM_THROTTLE(1, "Processing Elapsed: " << (detect_end - detect_start).toSec() << " sec");
            ROS_INFO_STREAM_THROTTLE(1, "Loop Speed: " << (curr_time - prev_loop_time).toSec() << " sec");
            prev_loop_time = curr_time;
        }

    }

    k4a_device_close(device);
    return EXIT_SUCCESS;


}