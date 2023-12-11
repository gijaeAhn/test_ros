//
// Created by sj on 23. 12. 6.
//


#include "ball_detection.h"
#include "shelf-pack.hpp"


cv::Mat moving_small;

void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    std::cout << "Freeing Image Memory. \n";
    exit(signum);
}

    cv::Mat k2cvMat(const k4a_image_t& input) {
        cv::Mat buf;
        auto format = k4a_image_get_format(input);
        std::cout << format << std::endl;
        if(format == K4A_IMAGE_FORMAT_COLOR_BGRA32 || format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
            return cv::Mat(k4a_image_get_height_pixels(input),
                           k4a_image_get_width_pixels(input),
                           CV_8UC4,
                           (void *)k4a_image_get_buffer(input));
        }
        else if(format == K4A_IMAGE_FORMAT_DEPTH16) {
            return cv::Mat(k4a_image_get_height_pixels(input),
                           k4a_image_get_width_pixels(input),
                           CV_16U,
                           (void *)k4a_image_get_buffer(input),
                           static_cast<size_t>(k4a_image_get_stride_bytes(input)));
        }
        return buf;
    }

    ros::Time kTime2Ros(const uint64_t& time) {
        // This will give INCORRECT timestamps until the first image.
        ros::Time ros_time;
        ros_time.fromNSec(time);
        return ros_time;
    }


    void fillCamInfo(_k4a_calibration_t& cali) {
        auto kCali = cali;

        int cam_width = kCali.color_camera_calibration.resolution_width;
        int cam_height = kCali.color_camera_calibration.resolution_height;

        //Resizing could be needed
        uint32_t resized_width = cam_width * debug_resize_w;
        uint32_t resized_height = cam_height * debug_resize_h;
        //

        //Get Params
        info_msg_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;     //Should be modified
        info_msg_->D.resize(5);
        info_msg_->D[0] = kCali.color_camera_calibration.intrinsics.parameters.param.k1;
        info_msg_->D[1] = kCali.color_camera_calibration.intrinsics.parameters.param.k2;
        info_msg_->D[2] = kCali.color_camera_calibration.intrinsics.parameters.param.k3;
        info_msg_->D[3] = kCali.color_camera_calibration.intrinsics.parameters.param.p1;
        info_msg_->D[4] = kCali.color_camera_calibration.intrinsics.parameters.param.p2;

        info_msg_->K.fill(0.0);
        info_msg_->K[0] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.fx);
        info_msg_->K[2] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.cx);
        info_msg_->K[4] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.fy);
        info_msg_->K[5] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.cy);
        info_msg_->K[8] = 1.0;
        info_msg_->R.fill(0.0);

        for (size_t i = 0; i < 3; i++) {
            info_msg_->R[i + i * 3] = 1;     // identity
        }

        //Shoulb be modified accroding to rotation and translation
        info_msg_->P.fill(0.0);
        info_msg_->P[0] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.fx);
        info_msg_->P[2] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.cx);
        info_msg_->P[6] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.fy);
        info_msg_->P[7] = static_cast<double>(kCali.color_camera_calibration.intrinsics.parameters.param.cy);
        info_msg_->P[10] = 1.0;

        info_msg_->width = debug_resize_w;
        info_msg_->height = debug_resize_h;
        info_msg_->header.frame_id= "camera_" + camera_id_;
    }

    //It depends on the camera mode so have to revise
    //In lowest resolution mode
    uint8_t getDepth(int x, int y, k4a_image_t& depth_image) {
        int depth = 0;
        auto width = k4a_image_get_width_pixels(depth_image);
        auto height = k4a_image_get_height_pixels(depth_image);
        if (x >= 0 && x < width && y >= 0 && y < height) {
//            auto depth_frame = k4a_image_get_buffer(depth_image);
            uint16_t* depth_frame = reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
            uint16_t depth = depth_frame[static_cast<int>(y*1024.0/720.0) * width + static_cast<int>(x*1024.0/1280.0)];
            return depth;
        }
        return 0;
    }

    geometry_msgs::Point getCartesianCoord(int x ,int y, uint16_t depth){

            double depth_in_real = 0.001 * depth;
            geometry_msgs::Point point;
            if(depth > 0)
            {
                printf(" fx fy %f %f cx cy %f %f", fx,fy,cx,cy);
                point.y = depth_in_real*(x-cx)/fx;
                point.z = depth_in_real*(y-cy)/fy;
                point.x = depth_in_real;
            }
            else
            {point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();}

            return point;

    }





    void cv_buffer_bookkeeping(const cv::Mat& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        cv_buffer.push_back(frame);
        if (cv_buffer.size() <= max_history) { return; }
        else { cv_buffer.pop_front(); }
    }

    void rgb_buffer_bookkeeping(const k4a_image_t& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        k4a_rgb_buffer.push_back(frame);
        if (k4a_rgb_buffer.size() <= max_history) { return; }
        else { k4a_rgb_buffer.pop_front(); }
    }

    void depth_buffer_bookkeeping(const k4a_image_t& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        k4a_depth_buffer.push_back(frame);
        if (k4a_depth_buffer.size() <= max_history) { return; }
        else { k4a_depth_buffer.pop_front(); }
    }

    //Segmentation fault error
    roi_list findMovingCandidates(const cv::Mat& frame) {
        auto start_time = ros::Time::now();



        //Resizing Frame
        if(resize_frame){
            cv::resize(frame,moving_small,cv::Size(), background_resize_,background_resize_,cv::INTER_AREA);
            if (!moving_small.empty()) {
                pBackSub_->apply(moving_small, moving_small, -1);
            }
        }
        else{
            pBackSub_->apply(frame,moving_small,-1);
        }

        debug_thresh_pub_.publish(cv_bridge::CvImage(info_msg_->header, "mono8", moving_small).toImageMsg(), info_msg_);
//        cv::cvtColor(moving_small,moving_small,cv::COLOR_BGR2GRAY);
        Contours contourlist;
        if(moving_small.type() == CV_8UC1){
            printf("Finding Contours Debug1\n");
            cv::findContours(moving_small, contourlist,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
        }
        std::deque<cv::Rect> ROIs;
        cv::Rect frame_roi(0,0,frame.cols,frame.rows);

        //Print out found contourlist
        printf("Size of contourlist : %ld\n", contourlist.size());
        for(const auto contour : contourlist) {
            auto area = cv::contourArea(contour);

            // Area size depends on location of camera
            if (area <=  5000 || 30000< area) continue;
            cv::Rect roi_small = cv::boundingRect(contour);
            //Modify depend on resizing
            background_resize_= 1.0;
            int pad = 2;  // Make sure to capture surrounding area
            cv::Rect roi;
            roi.x = (roi_small.x - pad) / background_resize_;
            roi.y = (roi_small.y - pad) / background_resize_;
            roi.width = (roi_small.width + 2 * pad) / background_resize_;
            roi.height = (roi_small.height + 2 * pad) / background_resize_;
            roi &= frame_roi;
            ROIs.push_back(roi);
            }
            printf("Size of ROIs : %ld\n", ROIs.size());

            // Merge all ROI that overlap
            bool overlap = ROIs.size() > 1;
            while(overlap) {
                // Get front and pop
                cv::Rect curr_roi = ROIs.front();
                ROIs.pop_front();
                // Check if curr roi is within another roi (if so then remove)
                // Else try to merge with any overlapping roi
                bool within = false;
                for (const auto& roi : ROIs) {
                    if ((curr_roi | roi) == roi) {
                        within = true;
                    }else if ((curr_roi & roi).area() > 0) {
                        curr_roi = (curr_roi | roi);
                    }
                }
                if (!within) {
                    ROIs.push_back(curr_roi);
                }
                // Check if any overlap
                overlap = false;
                for (int i = 0; i < ROIs.size() && !overlap; i++) {
                    for (int j = i+1; j < ROIs.size() && !overlap; j++) {
                        if ((ROIs[i] & ROIs[j]).area() > 0) {
                            overlap = true;
                        }
                    }
                }
            }
            roi_list unique_ROIs;
            for (const auto& roi : ROIs) {
                unique_ROIs.push_back(roi);
            }
            is_merged_ = false;
            if (print_diagnostics_)
                ROS_INFO_STREAM_THROTTLE(0.5,"Find Candidates: " << (ros::Time::now() - start_time).toSec() << " sec");
            return unique_ROIs;
        }





        void searchClosestCandidates(const cv::Mat& frame, roi_list& ROIs, cv::Rect ball_ROI) {
            // If previous ball is empty then leave
            if (prev_ball_ROI.empty()) { return; }

            // Just double-checking
            auto start = ros::Time::now();

            auto prev_center = (prev_ball_ROI.br() + prev_ball_ROI.tl())*0.5;
            cv::Rect closest;
            int center_min = INT_MAX;

            for (const auto& roi: ROIs) {
                auto roi_center = (roi.br() + roi.tl()) * 0.5;
                double dist = cv::norm(prev_center - roi_center);

                if (roi.br().x < (frame.cols / 2) && dist < center_min) {
                    center_min = dist;
                    closest = roi;
                }
                if (closest.empty()) { return; }
            }

            // Copying is much faster than performing other CV functions so just add to one image
            cv::Rect mask(0,0,closest.width, closest.height);

            // Should be broader
            cv::Mat broad(closest.height, closest.width , frame.type());
            frame(closest).copyTo(broad(mask));               // Cheap

            cv::cvtColor(broad, broad, cv::COLOR_BGR2HSV, 0);  // Very Expensive
            cv::inRange(broad, low_HSV_, high_HSV_, broad);    // Very Expensive
            int detect = cv::countNonZero(broad(mask));


            // Evaluate the guess
            auto evaluate_guess = [&](cv::Rect& prev_roi, cv::Rect& closest_roi, cv::Rect& ball_roi, int detection) {
                if (detection > 0) {             // If have color then go for it
                    ball_roi = closest_roi;
                    candidate_debug = DEBUG_GOOD;
                    is_merged_ |= (double) detection / closest_roi.area() < merged_percent_;
                } else {                                    // Else see if "similar"
                    double area_diff = abs(prev_roi.area()-closest_roi.area()) / (double) closest_roi.area();
                    double square_diff = abs(closest_roi.width-closest_roi.height) / (double) closest_roi.height;
                    double dist_diff = cv::norm(prev_roi.tl() - closest_roi.tl());

                    if (area_diff < guess_max_area_diff_
                        && square_diff < guess_max_square_diff_
                        && dist_diff < guess_max_dist_diff_) {
                        ball_roi = closest_roi;
                        candidate_debug = DEBUG_MAYBE;
                    } else {
                        candidate_debug = DEBUG_BAD;
                    }
                }
            };
            evaluate_guess(prev_ball_ROI, closest, ball_ROI, detect);

            if (print_diagnostics_)
                ROS_INFO_STREAM_THROTTLE(0.5,"Evaluate Closest Candidate: " << (ros::Time::now() - start).toSec() << " sec. "
                                                                            << "Success: " << (!ball_ROI.empty()) );
        }

        void searchCandidates(const cv::Mat& input_frame, roi_list& ROIs, cv::Rect& ball_ROI) {
            auto start = ros::Time::now();

            cv::Mat frame = input_frame;

            // Create 2D composite grid
            std::vector<Bin> bins;
            for (const auto &roi: ROIs) {
                bins.emplace_back(bins.size(), roi.width, roi.height);
            }
            ShelfPack::ShelfPackOptions options;
            options.autoResize = true;
            ShelfPack sprite(10, 10, options);
            std::vector<Bin *> results = sprite.pack(bins);

            // Copy data to shelf
            cv::Mat shelf((int) sprite.height(), (int) sprite.width(), frame.type());
            for (const auto &bin: results) {
                cv::Rect shelf_roi(cv::Point2f(bin->x, bin->y), ROIs[bin->id].size());
                frame(ROIs[bin->id]).copyTo(shelf(shelf_roi));
            }

            // HSV + Threshold + Detect
            cv::cvtColor(shelf, shelf, cv::COLOR_BGR2HSV, 0);
            cv::inRange(shelf, low_HSV_, high_HSV_, shelf);

            // Find non-zeros
            std::vector<cv::Point2f> detections;
            cv::findNonZero(shelf, detections);

            printf("Size of ball Detections %ld: \n", detections.size());

            // Find largest colored
            int max = INT_MIN;
            int cnt = INT_MIN;
            for (const auto &bin: results) {
                cv::Rect roi = ROIs[bin->id];
                cv::Rect bin_roi(cv::Point2f(bin->x, bin->y), roi.size());

                int count = 0;
                for (const auto &detection: detections) {
                    count += bin_roi.contains(detection) ? 1 : 0;
                }
                bool colored = count > 1;

                if (colored) {
                    int area = roi.area();
                    max = area;
                    cnt = count;
                    ball_ROI = roi;
                }
            }
        }

            geometry_msgs::PoseWithCovarianceStamped createEstimateMsg(const geometry_msgs::Point& position, ros::Time& time_taken) {
                // Create Pose Estimate
                geometry_msgs::PoseWithCovarianceStamped pose_msg;
                pose_msg.header.stamp = time_taken;
                pose_msg.header.frame_id = "camera_" + camera_id_ + "_center_camera_optical";

                pose_msg.pose.pose.position = position;
                pose_msg.pose.pose.orientation.w = 1;

                // Check
                double r = sqrt(pow(position.x, 2) + pow(position.y, 2) + pow(position.z, 2));
                if (r == 0 || position.x <= 0) {
                    ROS_ERROR_STREAM_THROTTLE(0.5, "Ball range or position.x == 0... Something is wrong!");
                }

                // Covariance x, y, z diag (dim: 6x6) (0, 6+1, 12+2)
                double d = position.z;
                covariance[0] = xy_k2*pow(d, 2) + xy_k1*d + xy_k0;; //0.01
                covariance[7] = xy_k2*pow(d, 2) + xy_k1*d + xy_k0;  //0.01
                covariance[14] = z_k2*pow(d, 2) + z_k1*d + z_k0;    //0.05
                std::copy(covariance.begin(), covariance.end(), pose_msg.pose.covariance.begin());

                return pose_msg;
            }


            void findCandidateCenter(const cv::Mat& frame,  cv::Rect& ball_ROI, cv::Point2i& center) {
                auto start = ros::Time::now();

                // Find the largest moving contour using history buffer
                    // Get center for image
                    auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2i& center, cv::Rect& ball_roi) {
                        std::vector<std::vector<cv::Point>> contours;
                        printf("Finding Contours Debug2\n");
                        cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                        std::vector<cv::Point> max_contour;
                        double max_area = INT_MIN;
                        for (const auto& contour : contours) {
                            double area = cv::contourArea(contour);
                            if(area < 35000 && area > 15000)
                            if (max_area < area) {
                                max_area = area;
                                max_contour = contour;
                            }
                        }
                        if (!max_contour.empty()) {
                            cv::Moments m = cv::moments(max_contour, true);
                            center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00) ;
                            printf(" X Y   %d %d \n", center.x, center.y);
                        }
                    };
                    calc_center(moving_small, ball_ROI.tl(), center,ball_ROI);
                if (print_diagnostics_)
                    ROS_INFO_STREAM_THROTTLE(0.5,"Find Center: " << (ros::Time::now() - start).toSec() << " sec");
            }

            void publishCenter(cv::Point2i& center, ros::Time& time_taken, k4a_image_t& depth_image) {
                // Publish Pose
                auto x = center.x;
                auto y = center.y;
                uint8_t depth = getDepth(x,y,depth_image);

                if (isnan(depth) || depth <= 0) {  return; }
                //Should implement getCartesianCoord(x,y)  x, y are pixel coord
                auto position = getCartesianCoord(x,y,depth);

                // Check for jumps
                static bool lockedIn = false;
                static ros::Time last_position_time;
                static geometry_msgs::Point prev_position;

                // Only publish if (not currently locked into position), (previous lockin reset), or (locked in and within distance)
                double dist = sqrt(pow(position.x-prev_position.x,2)+pow(position.y-prev_position.y,2)+pow(position.z-prev_position.z,2));
//              if (dist < INT_MAX) {
                auto pose_msg = createEstimateMsg(position, time_taken);
                printf(" X : %f  Y :%f Z : %f ",position.x,position.y,position.z);
                pose_pub_.publish(pose_msg);
//              }
                last_position_time = ros::Time::now();
                prev_position = position;
            }

            void publishDebug(const cv::Mat& frame, roi_list& ROIs, cv::Rect& ball_ROI, cv::Point2i buf_center, ros::Time time) {
                printf("Publishing Debug Image");
                // Only publish at fixed rate
                ros::Time last_publish_time = ros::Time::now();
//                if (ros::Time::now() - last_publish_time < ros::Duration(debug_publish_period_)) {
//                    return;
//                }
                last_publish_time = ros::Time::now();

                cv::Rect roi(0, 0, frame.cols, frame.rows);
                cv::Mat debug_img(frame);

                if (!debug_minimal_) {

                    // Draw all moving candidates
                    for (const auto& roi_buf : ROIs) {
                        if (roi_buf.contains(roi.br())) {
                            cv::add(debug_img(roi & roi_buf),  cv::Scalar(0, 50, 75), debug_img(roi & roi_buf), cv::noArray(), -1);
                        }
                    }

                    // Draw candidate
                    cv::Rect ball_roi = ball_ROI;
                    if (!ball_roi.empty()) {
                        cv::Scalar color = candidate_debug == DEBUG_GOOD  ? cv::Scalar( 0, 75, 0) :
                                           (candidate_debug == DEBUG_MAYBE ? cv::Scalar(75, 0,  0) :
                                            (candidate_debug == DEBUG_BAD   ? cv::Scalar( 0, 0, 75) :
                                             cv::Scalar(75, 0, 75)));
                        candidate_debug = DEBUG_NONE;
                        cv::add(frame(ball_roi & roi),  color, debug_img(ball_roi & roi), cv::noArray(), -1);
                    }

                    // Draw green dot
                    cv::Point2i center = buf_center;

                    if (center != cv::Point2i() && roi.contains(center)) {
                        printf("Draw Center\n");
                        int size = 7;
                        cv::Rect center_roi(center - cv::Point2i((size-1)/2, (size-1)/2), cv::Size(size,size));
                        cv::Scalar color = is_merged_ ? cv::Scalar(255, 50, 255) : cv::Scalar(255, 50, 50);
                        debug_img(center_roi & ball_roi).setTo(color);
                    }


                } else {
                    // Resize debug image
                    cv::resize(frame(roi), debug_img, cv::Size(), debug_resize_w, debug_resize_h, cv::INTER_LINEAR);
                    cv::Rect resized_roi(0, 0, debug_img.cols, debug_img.rows);

                    // Draw green dot
                    cv::Point2f center = center;
                    if (center != cv::Point2f() && roi.contains(center)) {
                        cv::Rect roi(cv::Point2i(center.x * debug_resize_w, center.y * debug_resize_h),  cv::Size(10, 10));
                        debug_img(roi & resized_roi).setTo(cv::Scalar(255, 50, 50));
                    }
                }

                info_msg_->header.stamp = time;
                debug_pub_.publish(cv_bridge::CvImage(info_msg_->header, "bgra8", debug_img).toImageMsg(), info_msg_);
            }


            void detectBall() {

                cv::Mat frame = cv_buffer.back();
                k4a_image_t k4a_depth_image = k4a_depth_buffer.back();
                ros::Time time_taken = kTime2Ros( (k4a_image_get_device_timestamp_usec(k4a_rgb_buffer.back())));
                // Get moving ROI candidates in shrunk image
                cv::Rect ball_ROI;
                roi_list ROIs = findMovingCandidates(frame); // Blocking
                // If have previous ball, find closest candidate and check if colored or very similar to previous (fast heuristic)
//                searchClosestCandidates(frame, ROIs, ball_ROI); // Blocking
                // If no ball candidate selected, pack candidates into new minimum area image and find largest colored candidate
                if (ball_ROI.empty() ) {
                    searchCandidates(frame, ROIs, ball_ROI);  // Blocking
                }
                prev_ball_ROI = ball_ROI;
                // If found candidates, perform background subtraction at high-resolution on candidate
                // If colored and moving parts strongly overlap, find center of moving part, else, colored part
                cv::Point2i center;
                if (!ball_ROI.empty()) {
                    printf("Ball ROI is not empty\n");
                    // Get High-Res moving ball
                    findCandidateCenter(frame, ball_ROI, center); // Blocking
                    if (center != cv::Point2i() ) {
                        printf("Ball Center Point is not empty\n");
                        publishCenter(center, time_taken, k4a_depth_image); // Blocking
                    }
                }
//                 Create debug image
                if (true) {
                    publishDebug(frame, ROIs, ball_ROI, center, time_taken); // (Non-Blocking)
                }


                debug_pub_.publish(cv_bridge::CvImage(info_msg_->header, "bgra8", frame).toImageMsg(), info_msg_);
            }












    // Why Covariance vector' size is 36?
