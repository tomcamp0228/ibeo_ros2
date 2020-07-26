/********************************************* 
 * Author: Pengfei Guo
 * Date: 2020-07-24 13:19:44
 * LastEditTime: 2020-07-24 14:01:05
 * LastEditors: Pengfei Guo
 * Description: Merge and calibrate point cloud
 * FilePath: /ibeo_ws/src/point_cloud_merge/src/point_cloud_merge.cpp
*********************************************/
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


// #include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
class point_cloud_merge: public rclcpp::Node{
    private:
    //Subscribers to receive point cloud
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Point_Sub_1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Point_Sub_2;
    //Publisher to publish merged point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Point_Merge_Pub;
};
int main(int argc, char** argv)
{
    return 0;
}