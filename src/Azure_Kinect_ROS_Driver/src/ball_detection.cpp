//
// Created by sj on 23. 12. 6.
//


#include "ball_detection.h"

namespace ball_detection {

    cv::Mat k2cvMat(const k4a::image& input) {
        cv::Mat buf(const k4a::image &input);
        return buf;
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


    double getDepth(double left, double right) {

    }

    void signalHandler( int signum){
        printf(" Interrupt siganl %d recieved.\n",signum);
        printf("Freeing Image Memory. \n");
        //Freeing Memory
    }

    void image_buffer_bookkeeping(const cv::Mat& frame) {
        // Have the latest <max_history> number of image saved in rolling queue
        img_buffer.push_back(frame);
        if (img_buffer.size() <= max_history) { return; }
        else { img_buffer.pop_front(); }
    }


    roi_list findMovingCandidates(const cv::Mat& frame){
        auto start_time = ros::Time::now();
        static cv::Mat moving_small;

        cv::resize(frame,moving_small,cv::Size(), background_resize_,background_resize_,cv::INTER_AREA);
        pBackSub_->apply(moving_small,moving_small,-1);
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

        void searchCandidates(const cv::Mat& frame, roi_list& ROIs, roi_pair& ball_ROIs) {
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
            int left_max = INT_MIN, right_max = INT_MIN;
            int left_cnt = INT_MIN, right_cnt = INT_MIN;
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
                    if (roi.br().x < frame.cols / 2 && area > left_max) {        // Left
                        left_max = area;
                        left_cnt = count;
                        ball_ROIs.first = roi;
                    } else if (roi.tl().x > frame.cols / 2 && area > right_max) { // Right
                        right_max = area;
                        right_cnt = count;
                        ball_ROIs.second = roi;
                    }
                }
            }
        }


            void findCandidateCenter(const cv::Mat& frame, roi_pair& ball_ROIs, point_pair& center) {
                // Just double checking
                auto start = ros::Time::now();

                // Find the largest moving contour using history buffer
                if (largest_moving_center_ && !is_merged_) {
                    // High-Res small-ROI moving area (non-blocking)
                    static cv::Mat left_moving, right_moving;
                    static cv::Mat left_moving_cpu, right_moving_cpu;
                    for (const auto& old_frame : img_buffer) {
                        pBackSub_left_->apply(old_frame(ball_ROIs.first), left_moving, -1);
                        pBackSub_right_->apply(old_frame(ball_ROIs.second), right_moving, -1);
                    }

                    // Get center for image
                    auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2f& center, cv::Rect& ball_roi) {
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
                    calc_center(left_moving, ball_ROIs.first.tl(), center.first, ball_ROIs.first);
                    calc_center(right_moving, ball_ROIs.second.tl(), center.second, ball_ROIs.second);

                } else {
                    auto&[left_roi, right_roi] = ball_ROIs;
                    cv::Rect left_mask(0,0,left_roi.width, left_roi.height);
                    cv::Rect right_mask(left_roi.width,0,right_roi.width, right_roi.height);

                    // Could use the one from evalution but oh well, maybe in the future
                    cv::Mat broad(std::max(left_roi.height, right_roi.height), left_roi.width + right_roi.width, frame.type());
                    frame(left_roi).copyTo(broad(left_mask));               // Cheap
                    frame(right_roi).copyTo(broad(right_mask));             // Cheap
                    cv::cvtColor(broad, broad, cv::COLOR_BGR2HSV, 0);  // Very Expensive
                    cv::inRange(broad, low_HSV_, high_HSV_, broad);    // Very Expensive

                    auto calc_center = [&](const cv::Mat& img, cv::Point2f parent_tl, cv::Point2f& center, cv::Rect& ball_roi) {
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
                    calc_center(broad(left_mask), left_roi.tl(), center.first, left_roi);
                    calc_center(broad(right_mask), right_roi.tl(), center.second, right_roi);
                }

                if (print_diagnostics_)
                    ROS_INFO_STREAM_THROTTLE(0.5,"Find Center: " << (ros::Time::now() - start).toSec() << " sec");
            }

            void publishCenter(point_pair& centers, ros::Time& time_taken) {
                // Publish Pose
                auto left_center = centers.first;
                auto right_center = centers.second;
                right_center.x -= img_buffer.front().cols / 2; // Since SideBySide, need to remove cols from right
                double depth = getDepth(left_center.x, right_center.x);
                if (isnan(depth) || depth <= 0) {  return; }
                auto position = getCartesianCoord(left_center.x, left_center.y);

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

            void publishDebug(const cv::Mat& frame, roi_list& ROIs, roi_pair& ball_ROIs, point_pair& centers, ros::Time time) {
                // Only publish at fixed rate
                static ros::Time last_publish_time = ros::Time::now();
                if (ros::Time::now() - last_publish_time < ros::Duration(debug_publish_period_)) {
                    return;
                }
                last_publish_time = ros::Time::now();

                cv::Rect left_roi(0, 0, frame.cols/2, frame.rows);
                static cv::Mat debug_img(frame(left_roi).size(), frame(left_roi).type());

                if (!debug_minimal_) {
                    // Copy current image
                    frame(left_roi).copyTo(debug_img);

                    // Draw all moving candidates
                    for (const auto& roi : ROIs) {
                        if (left_roi.contains(roi.br())) {
                            cv::add(debug_img(roi & left_roi),  cv::Scalar(0, 50, 75), debug_img(roi & left_roi), cv::noArray(), -1);
                        }
                    }

                    // Draw candidate
                    cv::Rect ball_roi = ball_ROIs.first;
                    if (!ball_roi.empty()) {
                        cv::Scalar color = candidate_debug == DEBUG_GOOD  ? cv::Scalar( 0, 75, 0) :
                                           (candidate_debug == DEBUG_MAYBE ? cv::Scalar(75, 0,  0) :
                                            (candidate_debug == DEBUG_BAD   ? cv::Scalar( 0, 0, 75) :
                                             cv::Scalar(75, 0, 75)));
                        candidate_debug = DEBUG_NONE;
                        cv::add(frame(ball_roi & left_roi),  color, debug_img(ball_roi & left_roi), cv::noArray(), -1);
                    }

                    // Draw green dot
                    cv::Point2f center = centers.first;
                    if (center != cv::Point2f() && left_roi.contains(center)) {
                        int size = 7;
                        cv::Rect center_roi(center - cv::Point2f((size-1)/2, (size-1)/2), cv::Size(size,size));
                        cv::Scalar color = is_merged_ ? cv::Scalar(255, 50, 255) : cv::Scalar(255, 50, 50);
                        debug_img(center_roi & left_roi).setTo(color);
                    }

                    cv::resize(debug_img, debug_img, cv::Size(), debug_resize_w, debug_resize_h, cv::INTER_AREA);

                } else {
                    // Resize debug image
                    cv::resize(frame(left_roi), debug_img, cv::Size(), debug_resize_w, debug_resize_h, cv::INTER_AREA);
                    cv::Rect resized_left_roi(0, 0, debug_img.cols, debug_img.rows);

                    // Draw green dot
                    cv::Point2f center = centers.first;
                    if (center != cv::Point2f() && left_roi.contains(center)) {
                        cv::Rect roi(cv::Point2f(center.x * debug_resize_w, center.y * debug_resize_h),  cv::Size(10, 10));
                        debug_img(roi & resized_left_roi).setTo(cv::Scalar(255, 50, 50));
                    }
                }

                info_msg_->header.stamp = time;
                debug_pub_.publish(cv_bridge::CvImage(info_msg_->header, "bgra8", debug_img).toImageMsg(), info_msg_);
            }

            void detectBall(int image_idx) {
                // sl::Mat -> cv::Mat (No Copy!)
                k4a::image k4a_frame = k4a_img_buffer[image_idx];
                cv::Mat frame = k2cvMat(k4a_frame);
                uint64_t buf_time = static_cast<uint64_t>(std::chrono::<std::chrono::nanoseconds>(k4a_frame.get_system_timestamp().count()));

                ros::Time time_taken(kTime2Ros(buf_time);

                // Image queue book-keeping
                image_buffer_bookkeeping(frame);

                // Get moving ROI candidates in shrunk image
                roi_pair ball_ROIs;
                roi_list ROIs = findMovingCandidates(frame); // Blocking

                // If have previous ball, find closest candidate and check if colored or very similar to previous (fast heuristic)
                searchClosestCandidates(frame, ROIs, ball_ROIs); // Blocking

                // If no ball candidate selected, pack candidates into new minimum area image and find largest colored candidate
                if (ball_ROIs.first.empty() || ball_ROIs.second.empty()) {
                    searchCandidates(frame, ROIs, ball_ROIs);  // Blocking
                }
                prev_ball_ROIs = ball_ROIs;

                // If found candidates, perform background subtraction at high-resolution on candidate
                // If colored and moving parts strongly overlap, find center of moving part, else, colored part
                point_pair centers;
                if (!ball_ROIs.first.empty() && !ball_ROIs.second.empty()) {
                    // Get High-Res moving ball
                    findCandidateCenter(frame, ball_ROIs, centers); // Blocking
                    if (centers.first != cv::Point2f() && centers.second != cv::Point2f()) {
                        publishCenter(centers, time_taken); // Blocking
                    }
                }

                // Create debug image
                if (debug_pub_.getNumSubscribers() != 0) {
                    publishDebug(frame, ROIs, ball_ROIs, centers, time_taken); // (Non-Blocking)
                }
            }




        }



    int main(int argc, char** argv)
    {

    }


    // Why Covariance vector' size is 36?
