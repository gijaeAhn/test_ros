#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>


void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)

{
ROS_INFO("Image received");
try
{cv_bridge::CvImagePtr Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
ROS_INFO("Distance : %.0f",Dest->image.at<float>(msg->height/2,msg->width/2));}
catch (cv_bridge::Exception& e)
{   ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return;}



}

int main(int argc, char **argv)

{

ros::init(argc, argv, "distance");

ros::NodeHandle n;

ROS_INFO("HELLO");

ros::Subscriber sub = n.subscribe("/camera/depth/image_rect_raw", 100, chatterCallback);

ros::spin();

return 0;

}