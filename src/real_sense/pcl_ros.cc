#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <typeinfo>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>

using Point = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

ros::Publisher pub;


void cloud_cb(const sensor_msgs::PointCloud2 input)
{   
    ros::Rate r(0.5);
    visualization_msgs::MarkerArray occu_marker;
    geometry_msgs::Point cubeCenter;
    PCLPointCloud pc;
    pcl::fromROSMsg(input,pc);


    int i = 0;

    visualization_msgs::Marker marker;
    for(PCLPointCloud::const_iterator it = pc.begin(); it !=pc.end() ;++it )
    {
        pcl::PointXYZ point(it->x,it->y,it->z);
        
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
        i++;          
    }
    ROS_INFO("Publishing Point %d",i);

    r.sleep();

}


// void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& input)
// {
//     pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//     pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//     pcl_conversions::toPCL(*input,*cloud);
//     pcl::PCLPointCloud2 cloud_filtered;

//     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//     sor.setInputCloud(cloudPtr);
//     sor.setLeafSize(1.0,1.0,1.0);
//     sor.filter(cloud_filtered);

//     sensor_msgs::PointCloud2 output;
//     pcl_conversions::fromPCL(cloud_filtered,output);

//     pub.publish(output);
    
// }
int main(int argc, char** argv)
{

    ros::init(argc,argv,"my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Rate r(0.5);

    pub = nh.advertise<visualization_msgs::MarkerArray>("output",10);
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1,cloud_cb);


    ros::spin();

    

    
    

}