#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define PI 3.14159265359

using namespace std;

ros::Publisher pub_roi;
ros::Publisher pub_transform;
ros::Publisher pub_downsampling;
ros::Publisher pub_center;
ros::Publisher pub_center_2;
ros::Publisher pub_center_3;
ros::Publisher pub_botton;
ros::Publisher pub_position_y;
ros::Publisher pub_position_x;

struct location {
    double x;
    double y;
};

double ROI_theta(double x, double y)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/PI;
    
    return theta;
}

bool path_complete = false;

void input_state(const std_msgs::StringConstPtr& msg)
{
    if (msg->data != "path_start")
    {
        path_complete = true;
    }

}

void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan, *cloud);

    //Transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    transform (0,0) = -0.9659;
    transform (0,2) = 0.2588;
    transform (2,0) = -0.2588;
    transform (2,2) = -0.9659;
    
    //transform (0,0) = -0.2588;
    //transform (0,2) = 0.9659;
    //transform (2,0) = -0.9659;
    //transform (2,2) = -0.2588;
    
    //transform (0,0) = 0.9659;
    //transform (0,2) = -0.2588;
    //transform (2,0) = -0.2588;
    //transform (2,2) = -0.9659;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    pcl::PCLPointCloud2 cloud_t;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_t);
    sensor_msgs::PointCloud2 utlidar_t;
    pcl_conversions::fromPCL(cloud_t, utlidar_t);
    utlidar_t.header.frame_id = "utlidar_lidar";
    pub_transform.publish(utlidar_t);

    pcl::PCLPointCloud2::Ptr cloud_t_ptr(new pcl::PCLPointCloud2());
    *cloud_t_ptr = cloud_t;
    
    /*
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_p);
    sensor_msgs::PointCloud2 velodyne_p;
    pcl_conversions::fromPCL(cloud_p, velodyne_p);
    */

    //DownSampling
    pcl::PCLPointCloud2 * cloud_d = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_d);
    pcl::PCLPointCloud2 cloud_filtered;

    sensor_msgs::PointCloud2ConstPtr utlidar_ptr(new sensor_msgs::PointCloud2(utlidar_t));
    pcl_conversions::toPCL(*utlidar_ptr, *cloud_d);

    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(cloudPtr);
    vox.setLeafSize (0.05, 0.05, 0.05);
    vox.filter(cloud_filtered);

    sensor_msgs::PointCloud2 downsampling;
    pcl_conversions::moveFromPCL(cloud_filtered, downsampling);
    downsampling.header.frame_id = "utlidar_lidar";
    pub_downsampling.publish(downsampling);

    //ROI
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(downsampling, laserCloudIn);

    location* y_arrary = new location[1000] {};
    int cnt = 0;
    double sum = 0;
    int point_num = 0;
    float max_x = 0;
    
    for(unsigned int i = 0; i < laserCloudIn.points.size(); i++)
    {
        double theta = ROI_theta(laserCloudIn.points[i].y, laserCloudIn.points[i].x);

        //ROS_INFO("theta:    %lf",theta);
        if((theta < 60 || theta > 120) ||
            (laserCloudIn.points[i].x < 0 || laserCloudIn.points[i].x > 2.5) ||
            (laserCloudIn.points[i].z < -0.2 || laserCloudIn.points[i].z > 0.2))
        {
            laserCloudIn.points[i].x = 0;
            laserCloudIn.points[i].y = 0;
            laserCloudIn.points[i].z = 0;
        }
        else{
            //ROS_INFO("X : %f\tY : %f\tZ: %f", laserCloudIn.points[i].x, laserCloudIn.points[i].y, laserCloudIn.points[i].z);
            //************************************************
            //Subscribe topic that announce navigation finish
            //*************************************************
            if (path_complete == true)
            {
                max_x = 0;
                path_complete = false;
            }
            if (laserCloudIn.points[i].x > max_x)
            {
                if (laserCloudIn.points[i].x > 2.5)
                {
                    ROS_INFO("[ERROR] There is no door\nx value:    %lf", laserCloudIn.points[i]);
                }
                else
                {
                    max_x = laserCloudIn.points[i].x;
                }
                //ROS_INFO("Elevator_distance: %f", max_x);
            }

            y_arrary[cnt].y = laserCloudIn.points[i].y;
            y_arrary[cnt].x = laserCloudIn.points[i].x;
            //ROS_INFO("Update : %f", y_arrary[cnt]);
            cnt += 1;
            point_num += 1;
        }
    }
    //ROS_INFO("Point : %d", point_num);

    pcl::PCLPointCloud2 cloud_roi;
    pcl::toPCLPointCloud2(laserCloudIn, cloud_roi);
    sensor_msgs::PointCloud2 utlidar_roi;
    pcl_conversions::fromPCL(cloud_roi, utlidar_roi);
    utlidar_roi.header.frame_id = "utlidar_lidar";
    pub_roi.publish(utlidar_roi);

    int n = 0;
    double small_y = 100;
    double large_y = 0;
    for (int i = 0; i < point_num; i++){
        if (y_arrary[i].y != 0){
            if (y_arrary[i].x < max_x*0.8)
            {
                continue;
            }
            else
            {
                if (y_arrary[i].y < small_y) small_y = y_arrary[i].y;
                if (y_arrary[i].y > large_y) large_y = y_arrary[i].y;
                sum += y_arrary[i].y;
                n += 1;
            }
        }
    }

    double center_data;
    center_data = ceil((sum / n) * 1000) / 1000;

    double center_2 = (large_y + small_y)/2;

    double center_3 = (center_data + center_2)/2;

    geometry_msgs::PointStamped center_location;
    center_location.point.x = max_x;
    center_location.point.y = center_data;
    center_location.header.frame_id = "utlidar_lidar";
    pub_center.publish(center_location);

    geometry_msgs::PointStamped center_location_2;
    center_location_2.point.x = max_x;
    center_location_2.point.y = center_2;
    center_location_2.header.frame_id = "utlidar_lidar";
    pub_center_2.publish(center_location_2);

    geometry_msgs::PointStamped center_location_3;
    center_location_3.point.x = max_x;
    center_location_3.point.y = center_3;
    center_location_3.header.frame_id = "utlidar_lidar";
    pub_center_3.publish(center_location_3);

    std_msgs::Float32 pos_y;
    pos_y.data = center_3;
    pub_position_y.publish(pos_y);
    std_msgs::Float32 pos_x;
    pos_x.data = max_x;
    pub_position_x.publish(pos_x);

    ROS_INFO("Elevator X: %lf   Elevator Y: %lf",max_x,center_3);

    delete [] y_arrary;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "utlidar_detection");
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar_point_merge", 100, input);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/utlidar/cloud", 100, input);
    ros::Subscriber sub_state = nh.subscribe<std_msgs::String> ("/path_state",100, input_state);
    pub_transform = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_points", 100);
    pub_downsampling = nh.advertise<sensor_msgs::PointCloud2> ("/down_sampling",100);
    pub_roi = nh.advertise<sensor_msgs::PointCloud2> ("/utlidar_roi",100);
    pub_center = nh.advertise<geometry_msgs::PointStamped> ("/door_center", 100);
    pub_center_2 = nh.advertise<geometry_msgs::PointStamped> ("/door_center_2", 100);
    pub_center_3 = nh.advertise<geometry_msgs::PointStamped> ("/door_center_3", 100);
    pub_position_y = nh.advertise<std_msgs::Float32> ("/door_y", 100);
    pub_position_x = nh.advertise<std_msgs::Float32> ("/door_x", 100);

    ros::spin();
}