#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <time.h>

#define PI 3.14159265359

using namespace std;

ros::Publisher pub_path;

struct position
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

float global_x = 0;
float global_y = 0;
float* y_array = new float[20];
float* x_array = new float[20];
int y_cnt = 0;
int x_cnt = 0;
float standard_x = 0;
float standard_y = 0;
bool door_open = false;
bool path_3_on = false;
bool path_5_on = false;
bool path_2_complete = false;
bool path_start_error = false;
int path_num = 0;
int open_cnt = 0;

void quickSort(float *data, int start, int end) {
  if (start >= end) return;
  int pivot = start;  // 기준 값
  int i = start + 1;
  int j = end;

  while (i <= j) {
    while (data[i] <=
           data[pivot])  // 키 값보다 큰 값 만날때까지 오른쪽으로 이동
      i++;
    while (data[j] >= data[pivot] &&
           j > start)  // 키 값보다 작은 값 만날 때까지 왼쪽으로 이동
      j--;
    if (i > j)  //현재 엇갈린 상태면 pivot 값 교체
    {
      float temp = data[j];
      data[j] = data[pivot];
      data[pivot] = temp;
    } else {
      float temp = data[j];
      data[j] = data[i];
      data[i] = temp;
    }
    // 재귀 호출
    quickSort(data, start, j - 1);
    quickSort(data, j + 1, end);
  }
}

void path_planning(position start, position goal)
{
    int point_num = 30;

    std::vector<geometry_msgs::PoseStamped> pose_stamp(point_num);

        for (int i = 0; i < point_num; i++)
        {
            pose_stamp.at(i).pose.position.x = start.x + i * ((goal.x - start.x) / 30.0);
            pose_stamp.at(i).pose.position.y = start.y + i * ((goal.y - start.y) / 30.0);
        }

        nav_msgs::Path path_plan;
        path_plan.poses = pose_stamp;
        path_plan.header.frame_id = "utlidar_lidar";
        pub_path.publish(path_plan);

}

void input_p(const geometry_msgs::PointStampedConstPtr& msg)
{
    if (x_cnt < 20)
    {
        x_array[x_cnt] = msg->point.x;
        quickSort(x_array,0,x_cnt);

        //for (int i = 0; i < x_cnt; i++) ROS_INFO("[%d]:     %lf\n",i,x_array[i]);
        x_cnt += 1;

    }
    else
    {
        if ((x_array[19] - x_array[10]) > (x_array[10] - x_array[0])) x_array[19] = msg->point.x;
        else x_array[0] = msg->point.x;
        quickSort(x_array,0,x_cnt);
        global_x = x_array[10];

        if (msg->point.x > 1.2 && path_num == 2)
        {
            open_cnt +=1;
            ROS_INFO("Open count up %lf", msg->point.x);
        }

        if (open_cnt >= 5)
        {
            door_open = true;
            open_cnt = 0;
        }
    }

    if (y_cnt < 20)
    {
        y_array[y_cnt] = msg->point.y;
        quickSort(y_array,0,y_cnt);

        //for (int i = 0; i < y_cnt; i++) ROS_INFO("[%d]:     %lf\n",i,y_array[i]);
        y_cnt += 1;

    }
    else
    {
        if ((y_array[19] - y_array[10]) > (y_array[10] - y_array[0])) y_array[19] = msg->point.y;
        else y_array[0] = msg->point.y;
        quickSort(y_array,0,y_cnt);
        global_y = y_array[10];
    }
}



void input_state(const std_msgs::StringConstPtr& msg)
{
    if (msg->data == "path_start" || path_start_error == true)
    {
        position start_pos;
        start_pos.y = 0;
        start_pos.x = 0;
        position botton_pos;
        botton_pos.y = global_y - 0.75;
        botton_pos.x = global_x - 0.85;
        standard_y = global_y;
        standard_x = global_x;

        if (global_x > 2.5)
        {
            path_start_error = true;
        }
        else
        {
            sleep(2);
            path_planning(start_pos, botton_pos);
            path_start_error = false;
        }
    }
    else if (msg->data == "path_1_complete")
    {
        path_num = 1;
    }
    else if (msg->data == "path_2_complete")
    {
        path_num = 2;
    }
    else if (msg->data == "path_3_complete")
    {
        path_num = 3;
    }
    else if (msg->data == "path_4_complete")
    {
        path_num = 4;
    }
    else if (msg->data == "path_5_complete")
    {
        path_num = 5;
    }
    else
    {
        if (path_num == 1 || path_num == 2)
        {
            path_num == 2;
        }
        else if (path_num == 4 || path_num == 5)
        {
            path_num == 5;
        }
        else
        {
            path_num = 0;
        }
        ROS_INFO("\nMoving\nPath Number: %d\n", path_num);
        ROS_INFO("\n==========================================================================");
    }

    ROS_INFO("Complete Path Number:  %d",path_num);



    if (path_num == 1)
    {
        position botton_pos;
        botton_pos.y = 0;
        botton_pos.x = 0;
        position init_pos;
        init_pos.y = 0.68;
        init_pos.x = 0;

        sleep(2);
        path_planning(botton_pos, init_pos);
    }

    

    else if (path_num == 2)
    {
        position init_pos;
        init_pos.y = 0;
        init_pos.x = 0;
        position elevator_pos;
        elevator_pos.y = 0;
        elevator_pos.x = 1.8;
        position e_botton_pos;
        e_botton_pos.y = -0.50;
        e_botton_pos.x = 1.8;
        
        path_2_complete = true;
        path_3_on = true;

        if (door_open == false)
        {
            ROS_INFO("Waiting -- Path_3: %d\nGlobal_x:    %lf", path_3_on, global_x);
        }
        else
        {
            ROS_INFO("Enter into Elevator");
            sleep(1);

            std::vector<geometry_msgs::PoseStamped> pose_stamp(30);
            for (int i = 0; i < 30; i++)
            {
                if (i < 24)
                {
                    pose_stamp.at(i).pose.position.x = init_pos.x + i * ((elevator_pos.x - init_pos.x) / 20.0);
                    pose_stamp.at(i).pose.position.y = init_pos.y + i * ((elevator_pos.y - init_pos.y) / 20.0);
                }
                else
                {
                    pose_stamp.at(i).pose.position.x = elevator_pos.x + (i-19) * ((e_botton_pos.x - elevator_pos.x) / 10.0);
                    pose_stamp.at(i).pose.position.y = elevator_pos.y + (i-19) * ((e_botton_pos.y - elevator_pos.y) / 10.0);
                }
            }

            nav_msgs::Path path_plan;
            path_plan.poses = pose_stamp;
            path_plan.header.frame_id = "utlidar_lidar";
            pub_path.publish(path_plan);

            door_open = false;
            path_3_on = false;
            ROS_INFO("Path 3 Published -- Path_3: %d",path_3_on);
        }       
    }
    else if (path_num == 3)
    {
        position e_botton_pos;
        e_botton_pos.y = 0;
        e_botton_pos.x = 0;
        position elevator_pos;
        elevator_pos.y = 0.65;
        elevator_pos.x = 0;

        sleep(2);
        path_planning(e_botton_pos, elevator_pos);
    }
    else if (path_num == 4)
    {
        position yaw_start;
        yaw_start.z = 0;
        position yaw_finish;
        yaw_finish.z = 180;
        
        std::vector<geometry_msgs::PoseStamped> pose_stamp(30);
        for (int i = 0; i < 30; i++)
        {
            pose_stamp.at(i).pose.position.z = yaw_start.z + i * ((yaw_finish.z - yaw_start.z) / 30.0);
        }

        nav_msgs::Path path_plan;
        path_plan.poses = pose_stamp;
        path_plan.header.frame_id = "utlidar_lidar";
        pub_path.publish(path_plan);
    }
    else if (path_num == 5)
    {
        position elevator_pos;
        elevator_pos.y = 0;
        elevator_pos.x = 0;
        position last_pos;
        last_pos.y = 0;
        last_pos.x = 2.0;

        path_5_on = true;

        if (door_open == false)
        {
            ROS_INFO("Waiting -- Path_5: %d\nGlobal_x:    %lf", path_5_on, global_x);
        }
        else
        {
            ROS_INFO("Get off the elevator");
            sleep(1);
            path_planning(elevator_pos, last_pos);
            path_5_on == false;
            ROS_INFO("Path 5 Published -- Path_5: %d",path_5_on);
        }
    }
    else if (path_num == 0)
    {
        ROS_INFO("Path num is 0 (First path or moving)");
    }
    else
    {
        ROS_INFO("Error");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "utlidar_path");
    ros::NodeHandle nh;
    ros::Subscriber sub_p = nh.subscribe<geometry_msgs::PointStamped> ("/door_center_2", 100, input_p);
    ros::Subscriber sub_state = nh.subscribe<std_msgs::String> ("/path_state",100, input_state);
    pub_path = nh.advertise<nav_msgs::Path> ("/local_path_1",100);

    ros::spin();
}
