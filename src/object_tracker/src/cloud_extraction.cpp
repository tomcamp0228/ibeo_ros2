// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <object_tracker/cloud_extraction.h>
#include <pcl_conversions/pcl_conversions.h>


class Cloud_Extraction :public rclcpp::Node
{
    private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planepub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scancloud;
    public:
    Cloud_Extraction():
    Node("cloud_extraction")
    {
        planepub=this->create_publisher<visualization_msgs::msg::MarkerArray>("pcl_extract",10);
        scancloud=this->create_subscription<sensor_msgs::msg::PointCloud2>("ibeo_scan_up",10,std::bind(&Cloud_Extraction::Callback,this,std::placeholders::_1));
    }
    visualization_msgs::msg::Marker createWireframeMarker(const float& x1, const float& y1,
                                                 const float& x2, const float& y2,
                                                 const float& x3, const float& y3,
                                                 const float& x4, const float& y4
                                                 ){

        visualization_msgs::msg::Marker box;
        geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;

        int size_z = 0.5;

        p1.x = x1;
        p1.y = y1;
        p1.z = size_z;

        p2.x = x2;
        p2.y = y2;
        p2.z = size_z;

        p3.x = x3;
        p3.y = y3;
        p3.z = size_z;
        
        p4.x = x4;
        p4.y = y4;
        p4.z = size_z;

        p5 = p1;
        p5.z = -size_z;
        p6 = p2;
        p6.z = -size_z;
        p7 = p3;
        p7.z = -size_z;
        p8 = p4;
        p8.z = -size_z;

        box.points.reserve(24);

        box.points.push_back(p1);
        box.points.push_back(p2);

        box.points.push_back(p2);
        box.points.push_back(p3);

        box.points.push_back(p3);
        box.points.push_back(p4);

        box.points.push_back(p4);
        box.points.push_back(p1);

        box.points.push_back(p1);
        box.points.push_back(p5);

        box.points.push_back(p2);
        box.points.push_back(p6);

        box.points.push_back(p3);
        box.points.push_back(p7);

        box.points.push_back(p4);
        box.points.push_back(p8);

        box.points.push_back(p5);
        box.points.push_back(p6);

        box.points.push_back(p6);
        box.points.push_back(p7);

        box.points.push_back(p7);
        box.points.push_back(p8);

        box.points.push_back(p8);
        box.points.push_back(p5);

        return box;
    }

    void Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg,*pcl_cloud);
        std::vector<object_tracker::PCLExtractBox> boxes = object_tracker::extractBox(pcl_cloud);
        RCLCPP_INFO(this->get_logger(),"SIZE: %i" , boxes.size());
        // ROS_INFO_STREAM("SIZE:" << boxes.size());
        visualization_msgs::msg::MarkerArray box_marker_list;
        uint i;
        
        for(std::vector<object_tracker::PCLExtractBox>::const_iterator it = boxes.begin (); it != boxes.end (); ++it){

            visualization_msgs::msg::Marker box = createWireframeMarker(
            it -> x[0], it -> y[0],
            it -> x[1], it -> y[1],
            it -> x[2], it -> y[2],
            it -> x[3], it -> y[3]
            );
            box.id = ++i;
            box.header.frame_id = "vehicle";
            box.type = visualization_msgs::msg::Marker::LINE_LIST;
            box.header.stamp = this->get_clock()->now();
            box.action = visualization_msgs::msg::Marker::ADD;
            box.lifetime = rclcpp::Duration(0,170000000);
            box.scale.x = 0.1;
            box.color.b = 1.0;
            box.color.a = 1.0;
            box_marker_list.markers.push_back(box);
        }
        planepub->publish(box_marker_list);
        
    }
};
//create a wire frame to represent detected object


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto cloud_extraction=std::make_shared<Cloud_Extraction>();
	

    rclcpp::spin(cloud_extraction);
    return 0;
    
}