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

bool local_start = false;
int path_complete_cnt = 0;

struct position {
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

void input_nav(const std_msgs::StringConstPtr& msg)
{
    if(msg->data == "reach_1_goal")
    {
        local_start = true;
    }
    ROS_INFO("\nLocal Path Mode:  %d", local_start);
}

void input_state(const std_msgs::StringConstPtr& msg)
{
    if (msg->data == "path_6_complete")
    {
        local_start = false;
    }
    path_complete_cnt += 1;
    ROS_INFO("\n[Path Number %d is start]", path_complete_cnt);
}

void input_point(const sensor_msgs::PointCloud2ConstPtr& scan)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan, *cloud);

    //Transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    //transform (0,0) = -0.9659;
    //transform (0,2) = 0.2588;
    //transform (2,0) = -0.2588;
    //transform (2,2) = -0.9659;

    transform (0,0) = 0.9659;
    transform (0,2) = -0.2588;
    transform (2,0) = -0.2588;
    transform (2,2) = -0.9659;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    pcl::PCLPointCloud2 cloud_t;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_t);
    sensor_msgs::PointCloud2 utlidar_t;
    pcl_conversions::fromPCL(cloud_t, utlidar_t);
    utlidar_t.header.frame_id = "utlidar_lidar";
    pub_transform.publish(utlidar_t);

    //DownSampling
    pcl::PCLPointCloud2::Ptr cloud_t_ptr(new pcl::PCLPointCloud2());
    *cloud_t_ptr = cloud_t;

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

    position* point_arrary = new position[1000] {};
    
    float max_x = 0;
    int point_num = 0;

    for(unsigned int i = 0; i < laserCloudIn.points.size(); i++)
    {
        double theta = ROI_theta(laserCloudIn.points[i].y, laserCloudIn.points[i].x);
        //ROS_INFO("Theta:    %lf",theta);

        if((theta < 60 || theta > 120) ||
            (laserCloudIn.points[i].x > 0 || laserCloudIn.points[i].x < -2.5) ||
            (laserCloudIn.points[i].z < -0.2 || laserCloudIn.points[i].z > 0.2))
        {
            laserCloudIn.points[i].x = 0;
            laserCloudIn.points[i].y = 0;
            laserCloudIn.points[i].z = 0;
        }
        else
        {
            if(local_start == true)
            {
                if(laserCloudIn.points[i].x < max_x)
                {
                    if (laserCloudIn.points[i].x < -2.5)
                    {
                        ROS_INFO("[ERROR] There is no door\nx value:    %lf", laserCloudIn.points[i]);
                    }
                    else
                    {
                        max_x = laserCloudIn.points[i].x;
                    }
                }

                point_arrary[point_num].y = laserCloudIn.points[i].y;
                point_arrary[point_num].x = laserCloudIn.points[i].x;
                point_num += 1;
            }
            else
            {
                ROS_INFO("\n[Local Mode doesn't start]\n");
            }
        }
    }

    pcl::PCLPointCloud2 cloud_roi;
    pcl::toPCLPointCloud2(laserCloudIn, cloud_roi);
    sensor_msgs::PointCloud2 utlidar_roi;
    pcl_conversions::fromPCL(cloud_roi, utlidar_roi);
    utlidar_roi.header.frame_id = "utlidar_lidar";
    pub_roi.publish(utlidar_roi);

    if (local_start == true)
    {
        int n = 0;
        double small_y = 100;
        double large_y = 0;
        double sum = 0;
        for (int i = 0; i < point_num; i++)
        {
            if (point_arrary[i].y != 0)
            {
                if (point_arrary[i].x < max_x*0.8)
                {
                    continue;
                }
                else
                {
                    if (point_arrary[i].y < small_y) small_y = point_arrary[i].y;
                    if (point_arrary[i].y > large_y) large_y = point_arrary[i].y;
                    sum += point_arrary[i].y;
                    n += 1;
                }
            }
        }

        double center_sum;
        center_sum = ceil((sum / n) * 1000) / 1000;

        double center_mid = (large_y + small_y)/2;

        geometry_msgs::PointStamped center_location;
        center_location.point.x = max_x;
        center_location.point.y = center_sum;
        center_location.header.frame_id = "utlidar_lidar";
        pub_center.publish(center_location);

        geometry_msgs::PointStamped center_location_2;
        center_location_2.point.x = max_x;
        center_location_2.point.y = center_mid;
        center_location_2.header.frame_id = "utlidar_lidar";
        pub_center_2.publish(center_location_2);

        ROS_INFO("elevator_x: %lf   elevator_y: %lf", max_x, center_mid);
    }

    delete [] point_arrary;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "utlidar_detection");
    ros::NodeHandle nh;
    ros::Subscriber sub_nav = nh.subscribe<std_msgs::String> ("/reach_goal",100, input_nav);
    ros::Subscriber sub_state = nh.subscribe<std_msgs::String> ("/path_state",100, input_state);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/utlidar/cloud", 100, input_point);
    
    pub_transform = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_points", 100);
    pub_downsampling = nh.advertise<sensor_msgs::PointCloud2> ("/down_sampling",100);
    pub_roi = nh.advertise<sensor_msgs::PointCloud2> ("/utlidar_roi",100);
    pub_center = nh.advertise<geometry_msgs::PointStamped> ("/door_center", 100);
    pub_center_2 = nh.advertise<geometry_msgs::PointStamped> ("/door_center_2", 100);

    ros::spin();
}