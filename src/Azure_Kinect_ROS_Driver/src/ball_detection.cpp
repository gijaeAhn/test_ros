//
// Created by sj on 23. 12. 6.
//


#include "ball_detection.h"
#include "shelf-pack.hpp"


void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    std::cout << "Freeing Image Memory. \n";
    exit(signum);
}

    cv::Mat k2cvMat(const k4a::image& input) {
//        auto format = input.get_format();
        std::cout<< "Debug1.7" << std::endl;
//        std::cout << format << std::endl;

        auto frame = input.handle();
//        if(format == K4A_IMAGE_FORMAT_COLOR_BGRA32 || format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
//            cv::Mat cImg = cv::Mat(k4a_image_get_height_pixels(frame), k4a_image_get_width_pixels(frame), CV_8UC4,
//                                   k4a_image_get_buffer(frame));
            std::cout<< "COLOR" << std::endl;
//            return cImg;
            return cv::Mat(input.get_height_pixels(),
                           input.get_width_pixels(),
                           CV_8UC4,
                           (void *)input.get_buffer());
//        }
//        else if(format == K4A_IMAGE_FORMAT_DEPTH16) {
//            std::cout<< "DEPTH" << std::endl;
//            return cv::Mat(input.get_height_pixels(),
//                           input.get_width_pixels(),
//                           CV_16U,
//                           (void *)input.get_buffer(),
//                           static_cast<size_t>(input.get_stride_bytes()));
////            cv::Mat dImg = cv::Mat(k4a_image_get_height_pixels(frame), k4a_image_get_width_pixels(frame),)
//        }
    }

    ros::Time kTime2Ros(uint64_t time) {
        uint32_t sec = static_cast<uint32_t>(time / 1000000);
        uint32_t nsec = static_cast<uint32_t>((time % 1000000) * 1000);
        ros::Time rostime(sec, nsec);
        return rostime;
    }


    void fillCamInfo(k4a::calibration cali) {
        auto kCali = cali.depth_camera_calibration;

        int cam_width = kCali.resolution_width;
        int cam_height = kCali.resolution_height;

        //Resizing could be needed
        uint32_t resized_width = cam_width * debug_resize_w;
        uint32_t resized_height = cam_height * debug_resize_h;
        //

        //Get Params
        info_msg_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;     //Should be modified
        info_msg_->D.resize(5);
        info_msg_->D[0] = kCali.intrinsics.parameters.param.k1;
        info_msg_->D[1] = kCali.intrinsics.parameters.param.k2;
        info_msg_->D[2] = kCali.intrinsics.parameters.param.k3;
        info_msg_->D[3] = kCali.intrinsics.parameters.param.p1;
        info_msg_->D[4] = kCali.intrinsics.parameters.param.p2;

        info_msg_->K.fill(0.0);
        info_msg_->K[0] = static_cast<double>(kCali.intrinsics.parameters.param.fx);
        info_msg_->K[2] = static_cast<double>(kCali.intrinsics.parameters.param.cx);
        info_msg_->K[4] = static_cast<double>(kCali.intrinsics.parameters.param.fy);
        info_msg_->K[5] = static_cast<double>(kCali.intrinsics.parameters.param.cy);
        info_msg_->K[8] = 1.0;
        info_msg_->R.fill(0.0);

        for (size_t i = 0; i < 3; i++) {
            info_msg_->R[i + i * 3] = 1;     // identity
        }

        //Shoulb be modified accroding to rotation and translation
        info_msg_->P.fill(0.0);
        info_msg_->P[0] = static_cast<double>(kCali.intrinsics.parameters.param.fx);
        info_msg_->P[2] = static_cast<double>(kCali.intrinsics.parameters.param.cx);
        info_msg_->P[6] = static_cast<double>(kCali.intrinsics.parameters.param.fy);
        info_msg_->P[7] = static_cast<double>(kCali.intrinsics.parameters.param.cy);
        info_msg_->P[10] = 1.0;

        info_msg_->width = debug_resize_w;
        info_msg_->height = debug_resize_h;
        info_msg_->header.frame_id= "camera_" + camera_id_;
    }

    //it depends on the camera mode so have to revise
    //In lowest resolution mode
    uint8_t getDepth(int x, int y, k4a::image& depth_image) {
        int depth = 0;
        auto width = depth_image.get_width_pixels();
        auto height = depth_image.get_height_pixels();
        if (x >= 0 && x < depth_image.get_width_pixels() && y >= 0 && y < depth_image.get_height_pixels()) {
            auto depth_frame = depth_image.handle();
            uint8_t *depth_image_buffer = k4a_image_get_buffer(depth_frame);
            uint8_t depth = depth_image_buffer[y * width + x];
            return depth;
        }
    }

    geometry_msgs::Point getCartesianCoord(int x ,int y, uint8_t depth){

            double depth_in_real = 0.0;
            geometry_msgs::Point point;
            if(depth > 0)
            {
                point.x = depth_in_real*(x-cx)/fx;
                point.y = depth_in_real*(y-cy)/fy;
                point.z = depth_in_real;
            }
            else
            {point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();}

            return point;

    }





    void cv_buffer_bookkeeping(const cv::Mat& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        img_buffer.push_back(frame);
        if (img_buffer.size() <= max_history) { return; }
        else { img_buffer.pop_front(); }
    }

    void rgb_buffer_bookkeeping(const k4a::image& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        k4a_rgb_buffer.push_back(frame);
        if (k4a_rgb_buffer.size() <= max_history) { return; }
        else { k4a_rgb_buffer.pop_front(); }
    }

    void depth_buffer_bookkeeping(const k4a::image& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        k4a_depth_buffer.push_back(frame);
        if (k4a_depth_buffer.size() <= max_history) { return; }
        else { k4a_depth_buffer.pop_front(); }
    }

    //Segmentation fault error
    roi_list findMovingCandidates(const cv::Mat& frame){
        printf("frame size : %d, %d", frame.cols,frame.rows);
        if(frame.empty()){
            printf("Empty frame");
        }
        printf("Background resize :  %f", background_resize_);
        auto start_time = ros::Time::now();
        cv::Mat moving_small;
        printf("moving small size : %d, %d", moving_small.cols,moving_small.rows);


        cv::resize(frame,moving_small,cv::Size(), background_resize_,background_resize_,cv::INTER_AREA);
        std::cout<< "Debug2.1" << std::endl;
        pBackSub_->apply(moving_small,moving_small,-1);
        std::cout<< "Debug2.2" << std::endl;
        Contours contourlist;
        cv::findContours(moving_small, contourlist,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
        std::deque<cv::Rect> ROIs;
        cv::Rect frame_roi(0,0,frame.cols,frame.rows);
        for(const auto contour : contourlist) {
            auto area = cv::contourArea(contour);
            if (area <= 1 || moving_small.size().area() / 4.0 < area) continue;

            cv::Rect roi_small = cv::boundingRect(contour);

            int pad = 2;  // Make sure to capture surrounding area
            cv::Rect roi;
            roi.x = (roi_small.x - pad) / background_resize_;
            roi.y = (roi_small.y - pad) / background_resize_;
            roi.width = (roi_small.width + 2 * pad) / background_resize_;
            roi.height = (roi_small.height + 2 * pad) / background_resize_;
            roi &= frame_roi;

            ROIs.push_back(roi);
            }
        std::cout<< "Debug2.3" << std::endl;

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
                        curr_roi = (curr_roi | roi) & frame_roi;
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

            auto prev_center = prev_ball_ROI.br() + prev_ball_ROI.tl()*0.5;
            cv::Rect closest;
            int center_min = INT_MAX;

            for (const auto& roi: ROIs) {
                auto roi_center = (roi.br() + roi.tl()) * 0.5;
                double dist = cv::norm(prev_center - roi_center);

                if (roi.br().x < frame.cols / 2 && dist < center_min) {
                    center_min = dist;
                    closest = roi;
                }
                if (closest.empty()) { return; }
            }

            // Idk why this is faster than doing individually
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

        void searchCandidates(const cv::Mat& frame, roi_list& ROIs, cv::Rect& ball_ROI) {
            // I just waited before this so I am going to block....
            auto start = ros::Time::now();

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

            // Find largest colored left and right
            int max = INT_MIN ;
            int cnt = INT_MIN ;
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
//                    if (roi.br().x < frame.cols / 2 && area > left_max) {        // Left
//                        left_max = area;
//                        left_cnt = count;
//                        ball_ROIs.first = roi;
//                    } else if (roi.tl().x > frame.cols / 2 && area > right_max) { // Right
//                        right_max = area;
//                        right_cnt = count;
//                        ball_ROIs.second = roi;
//                    }
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
                if (r == 0 || position.z <= 0) {
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
                // Just double checking
                auto start = ros::Time::now();

                // Find the largest moving contour using history buffer
                if (largest_moving_center_ && !is_merged_) {
                    // High-Res small-ROI moving area (non-blocking)
//                    static cv::Mat left_moving, right_moving;
//                    static cv::Mat left_moving_cpu, right_moving_cpu;
                    static cv::Mat moving;
                    for (const auto& old_frame : img_buffer) {
                    //For Stereo Camera
//                        pBackSub_left_->apply(old_frame(ball_ROIs.first), left_moving, -1);
//                        pBackSub_right_->apply(old_frame(ball_ROIs.second), right_moving, -1);
                        pBackSub_->apply(old_frame(ball_ROI),moving,-1);
                    }

                    // Get center for image
                    auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2i& center, cv::Rect& ball_roi) {
                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                        std::vector<cv::Point> max_contour;
                        double max_area = INT_MIN;
                        for (const auto& contour : contours) {
                            double area = cv::contourArea(contour);
                            if (max_area < area) {
                                max_area = area;
                                max_contour = contour;
                            }
                        }
                        if (!max_contour.empty()) {
                            cv::Moments m = cv::moments(max_contour, true);
                            center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00) + parent_tl;
                        }
                    };
//                    calc_center(left_moving, ball_ROIs.first.tl(), center.first, ball_ROIs.first);
//                    calc_center(right_moving, ball_ROIs.second.tl(), center.second, ball_ROIs.second);
                    calc_center(moving, ball_ROI.tl(), center,ball_ROI);

                    // Else is for merging Stereo Camera images
                } else {
//                    auto&[left_roi, right_roi] = ball_ROIs;
//                    cv::Rect left_mask(0,0,left_roi.width, left_roi.height);
//                    cv::Rect right_mask(left_roi.width,0,right_roi.width, right_roi.height);
                    cv::Rect mask (0,0,ball_ROI.width,ball_ROI.height);

                    // Could use the one from evalution but oh well, maybe in the future
                    // Should be Broader
                    cv::Mat broad(std::max(ball_ROI.height, ball_ROI.height), ball_ROI.width + ball_ROI.width, frame.type());
//                    frame(left_roi).copyTo(broad(left_mask));               // Cheap
//                    frame(right_roi).copyTo(broad(right_mask));             // Cheap
                    frame(ball_ROI).copyTo(broad(mask));
                    cv::cvtColor(broad, broad, cv::COLOR_BGR2HSV, 0);  // Very Expensive
                    cv::inRange(broad, low_HSV_, high_HSV_, broad);    // Very Expensive

                    auto calc_center = [&](const cv::Mat& img, cv::Point2i parent_tl, cv::Point2i& center, cv::Rect& ball_roi) {
                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
                        std::vector<cv::Point> max_contour;
                        double max_area = INT_MIN;
                        for (const auto& contour : contours) {
                            double area = cv::contourArea(contour);
                            if (max_area < area) {
                                max_area = area;
                                max_contour = contour;
                            }
                        }
                        if (!max_contour.empty()) {
                            cv::Moments m = cv::moments(max_contour, true);
                            center = cv::Point2i(m.m10 / m.m00, m.m01 / m.m00) + parent_tl;
                        }
                    };
//                    calc_center(broad(left_mask), left_roi.tl(), center.first, left_roi);
//                    calc_center(broad(right_mask), right_roi.tl(), center.second, right_roi);
                    calc_center(broad(mask),ball_ROI.tl(),center,ball_ROI);
                }

                if (print_diagnostics_)
                    ROS_INFO_STREAM_THROTTLE(0.5,"Find Center: " << (ros::Time::now() - start).toSec() << " sec");
            }

            void publishCenter(cv::Point2i& center, ros::Time& time_taken, k4a::image& depth_image) {
                // Publish Pose
//                auto left_center = centers.first;
//                auto right_center = centers.second;
//                right_center.x -= img_buffer.front().cols / 2; // Since SideBySide, need to remove cols from right
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
                if (!lockedIn
                    || (ros::Time::now() - last_position_time) > ros::Duration(lockin_wait_)
                    || (lockedIn && dist < lockin_max_dist_)) {
                    // I kinda don't trust the first one, might be a fluke
                    if (lockedIn) {
                        auto pose_msg = createEstimateMsg(position, time_taken);
                        pose_pub_.publish(pose_msg);
                    }
                    last_position_time = ros::Time::now();
                    prev_position = position;
                    lockedIn = true;
                }
            }

            void publishDebug(const cv::Mat& frame, roi_list& ROIs, cv::Rect& ball_ROI, cv::Point2i buf_center, ros::Time time) {
                // Only publish at fixed rate
                static ros::Time last_publish_time = ros::Time::now();
                if (ros::Time::now() - last_publish_time < ros::Duration(debug_publish_period_)) {
                    return;
                }
                last_publish_time = ros::Time::now();

                cv::Rect roi(0, 0, frame.cols, frame.rows);
                static cv::Mat debug_img(frame(roi).size(), frame(roi).type());

                if (!debug_minimal_) {
                    // Copy current image
                    frame(roi).copyTo(debug_img);

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
                        int size = 7;
                        cv::Rect center_roi(center - cv::Point2i((size-1)/2, (size-1)/2), cv::Size(size,size));
                        cv::Scalar color = is_merged_ ? cv::Scalar(255, 50, 255) : cv::Scalar(255, 50, 50);
                        debug_img(center_roi & ball_roi).setTo(color);
                    }

                    cv::resize(debug_img, debug_img, cv::Size(), debug_resize_w, debug_resize_h, cv::INTER_AREA);

                } else {
                    // Resize debug image
                    cv::resize(frame(roi), debug_img, cv::Size(), debug_resize_w, debug_resize_h, cv::INTER_AREA);
                    cv::Rect resized_roi(0, 0, debug_img.cols, debug_img.rows);

                    // Draw green dot
                    cv::Point2f center = center;
                    if (center != cv::Point2f() && roi.contains(center)) {
                        cv::Rect roi(cv::Point2f(center.x * debug_resize_w, center.y * debug_resize_h),  cv::Size(10, 10));
                        debug_img(roi & resized_roi).setTo(cv::Scalar(255, 50, 50));
                    }
                }

                info_msg_->header.stamp = time;
                debug_pub_.publish(cv_bridge::CvImage(info_msg_->header, "bgra8", debug_img).toImageMsg(), info_msg_);
            }


            void detectBall(int image_idx) {

                std::cout<< "Debug1" << std::endl;
                k4a::image k4a_rgb_frame = k4a_rgb_buffer.back();
                cv::Mat frame = img_buffer.back();
                printf("Frame size : %d , %d\n", frame.cols, frame.rows);
                std::cout<< "Debug2" << std::endl;
                k4a::image k4a_depth_image = k4a_depth_buffer.back();
                uint64_t buf_time = k4a_rgb_frame.get_system_timestamp().count();

                std::cout<< "Debug3" << std::endl;

                ros::Time time_taken = kTime2Ros(buf_time);

                // Image queue book-keeping
                cv_buffer_bookkeeping(frame);

                std::cout<< "Debug4" << std::endl;

                printf("Frame size : %d , %d\n", frame.cols, frame.rows);
                // Get moving ROI candidates in shrunk image
                cv::Rect ball_ROI;
                std::cout<< "Debug5" << std::endl;

                roi_list ROIs = findMovingCandidates(frame); // Blocking


                // If have previous ball, find closest candidate and check if colored or very similar to previous (fast heuristic)
                searchClosestCandidates(frame, ROIs, ball_ROI); // Blocking

                // If no ball candidate selected, pack candidates into new minimum area image and find largest colored candidate
                if (ball_ROI.empty() || ball_ROI.empty()) {
                    searchCandidates(frame, ROIs, ball_ROI);  // Blocking
                }
                prev_ball_ROI = ball_ROI;

                std::cout<< "Debug6" << std::endl;

                // If found candidates, perform background subtraction at high-resolution on candidate
                // If colored and moving parts strongly overlap, find center of moving part, else, colored part
                cv::Point2i center;
                if (!ball_ROI.empty() && !ball_ROI.empty()) {
                    // Get High-Res moving ball
                    findCandidateCenter(frame, ball_ROI, center); // Blocking
                    if (center != cv::Point2i() ) {
                        publishCenter(center, time_taken, k4a_depth_image); // Blocking
                    }
                }

                std::cout<< "Debug7" << std::endl;

                // Create debug image
                if (debug_pub_.getNumSubscribers() != 0) {
                    publishDebug(frame, ROIs, ball_ROI, center, time_taken); // (Non-Blocking)
                }
            }












    // Why Covariance vector' size is 36?
