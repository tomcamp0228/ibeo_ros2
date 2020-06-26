//======================================================================
/*! \file obj_visualization.cpp
 *
 * \copydoc Copyright
 * \author Kexin Zheng
 * \date April 17, 2019
 *
 * Use Object data in ibeo_8l_msgs to demonstrate visualization
 *///-------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <ibeo_8l_msgs/msg/object_lux_ros.hpp>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include <sstream>
#include <memory>

//======================================================================
// ros::Publisher objectcpts_vis_pub;
// ros::Publisher object_vis_pub;
// rclcpp::Node::SharedPtr nh;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objectcpts_vis_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_vis_pub;
rclcpp::Node::SharedPtr node;
//======================================================================
//生成以某点为中心的线框盒
//======================================================================
visualization_msgs::msg::Marker createWireframeMarker(const float& center_x,	const float& center_y,
												float size_x,	float size_y,	const float& size_z){

	visualization_msgs::msg::Marker box;
	box.pose.position.x = center_x;
	box.pose.position.y = center_y;
	box.pose.position.z = 0.0;
	geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;

	size_y = (size_y <= 0.1f) ? 0.1f : size_y;
	size_x = (size_x <= 0.1f) ? 0.1f : size_x;

	float half_x = (0.5) * size_x;
	float half_y = (0.5) * size_y;

	p1.x = half_x;
	p1.y = half_y;
	p1.z = size_z;
	p2.x = half_x;
	p2.y = -half_y;
	p2.z = size_z;
	p3.x = -half_x;
	p3.y = -half_y;
	p3.z = size_z;
	p4.x = -half_x;
	p4.y = half_y;
	p4.z = size_z;
	p5 = p1;
	p5.z = 0.0;
	p6 = p2;
	p6.z = 0.0;
	p7 = p3;
	p7.z = 0.0;
	p8 = p4;
	p8.z = 0.0;

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

//======================================================================
//由点集生成包围框
//======================================================================
visualization_msgs::msg::Marker createContourFrameMarker(const std::vector<ibeo_8l_msgs::msg::Point2Df>& pts,rclcpp::Node::SharedPtr node){

	visualization_msgs::msg::Marker marker;
	marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
	marker.lifetime = rclcpp::Duration(0,170000000);
	marker.color.a = 0.5;
	marker.color.r = 1.0; marker.color.g = marker.color.b = 0;
	marker.frame_locked = false;
	marker.scale.x = 0.05;
	marker.action = visualization_msgs::msg::Marker::MODIFY;
	marker.header.stamp =node->get_clock()->now();

	marker.points.reserve(pts.size());
	// int i;
	geometry_msgs::msg::Point p1, p2;

	p1.x = p2.x = pts[0].x;
	p1.y = p2.y = pts[0].y;
	p1.z = 0.5; p2.z = -0.2;
	marker.points.push_back(p1);
	marker.points.push_back(p2);
	// for(i = 0; i < pts.size(); i ++){
	// 	p1.x = pts[i].getX();
	// 	p1.y = pts[i].getY();
	// 	marker.points.push_back(p1);
	// }
	return marker;
}

//======================================================================
//回调函数，收到ObjectListLuxRos时，发送可视化
//======================================================================
void visualCallback(const ibeo_8l_msgs::msg::ObjectListLuxRos::SharedPtr objList){

	const std::vector< ibeo_8l_msgs :: msg :: ObjectLuxRos > &objs = objList -> object_list;

	int i, n = objList -> number_of_objects;
	//create the array-type msg for markers and object info
	visualization_msgs :: msg :: MarkerArray marker_array;
	//create the array-tpye msg for contour points of objects
	visualization_msgs :: msg :: MarkerArray cpts_marker_array;

	for(i = 0; i < n; i++){

		//create the object box vislization msg
		visualization_msgs :: msg :: Marker marker_box = createWireframeMarker(objs[i].object_box_center.x, objs[i].object_box_center.y,
												objs[i].object_box_size.x, objs[i].object_box_size.y, 1.2);

		marker_box.id = objs[i].id;
		//tf2::Quaternion qn = tf::createQuaternionFromYaw(objs[i].object_box_orientation);
		tf2::Quaternion qn;
		qn.setRPY(0,0,objs[i].object_box_orientation);
		marker_box.pose.orientation.x = qn.x();
		marker_box.pose.orientation.y = qn.y();
		marker_box.pose.orientation.z = qn.z();
		marker_box.pose.orientation.w = qn.w();
		marker_box.lifetime = rclcpp::Duration(0,170000000);
		marker_box.color.a = 0.5;
		marker_box.color.r = marker_box.color.g = marker_box.color.b = 1.0;
		marker_box.frame_locked = false;
		marker_box.scale.x = 0.05;
		marker_box.scale.y = 0.05;
		marker_box.scale.z = 0.05;
		marker_box.type = visualization_msgs::msg::Marker::LINE_LIST;
		marker_box.action = visualization_msgs::msg::Marker::ADD;
		marker_box.header.stamp = node->get_clock()->now();
		marker_box.header.frame_id = objList -> header.frame_id;

		//fill the label text with classification
		std::string label;
		switch (objs[i].classification)
		{
			case ibeo_8l_msgs::msg::ObjectLuxRos::UNCLASSIFIED:
				label = "Unclassified";
				// Unclassified - white
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::UNKNOWN_SMALL:
				label = "Unknown Small";
				// Unknown small - blue
				marker_box.color.r = marker_box.color.g = 0;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::UNKNOWN_BIG:
				label = "Unknown Big";
				// Unknown big - dark blue
				marker_box.color.r = marker_box.color.g = 0;
				marker_box.color.b = 0.5;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::PEDESTRIAN:
				label = "Pedestrian";
				// Pedestrian - red
				marker_box.color.g = marker_box.color.b = 0;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::BIKE:
				label = "Bike";
				// Bike - dark red
				marker_box.color.g = marker_box.color.b = 0;
				marker_box.color.r = 0.5;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::BICYCLE:
				label = "Bicycle";
				// Bicycle - dark red
				marker_box.color.g = marker_box.color.b = 0;
				marker_box.color.r = 0.5;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::CAR:
				label = "Car";
				// Car - green
				marker_box.color.b = marker_box.color.r = 0;
				break;
			case ibeo_8l_msgs::msg::ObjectLuxRos::TRUCK:
				label = "Truck";
				// Truck - dark green
				marker_box.color.b = marker_box.color.r = 0;
				marker_box.color.g = 0.5;
				break;
			default:
				label = "Unknown";
				marker_box.color.r = marker_box.color.b = marker_box.color.g = 0.0;
				break;
		}
		marker_box.ns = label;

		//create label visualization msg to label the object
		visualization_msgs::msg::Marker marker_label;
		marker_label.id  = objs[i].id + 1000;	//to avoid conflict of ID
		marker_label.ns = label;
		marker_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		marker_label.action = visualization_msgs::msg::Marker::ADD;
		marker_label.pose.position.x = objs[i].object_box_center.x;
		marker_label.pose.position.y = objs[i].object_box_center.y;
		marker_label.pose.position.z = 0.5;
		marker_label.text = label + ", id: " + std::to_string(objs[i].id) + " velocity: " +std::to_string(sqrt(objs[i].absolute_velocity.x*objs[i].absolute_velocity.x+objs[i].absolute_velocity.y*objs[i].absolute_velocity.y));
		marker_label.scale.x = 0.1;
		marker_label.scale.y = 0.1;
		marker_label.scale.z = 0.5;
		marker_label.lifetime = marker_box.lifetime;
		marker_label.color.r = marker_label.color.g = marker_label.color.b = 1;
		marker_label.color.a = 0.5;
		marker_label.header.stamp = node->get_clock()->now();
		marker_label.header.frame_id = objList -> header.frame_id;

		//create contour points visualization msg
		if(objs[i].number_of_contour_points != 0){
			const std::vector< ibeo_8l_msgs::msg::Point2Df > &contourpts = objs[i].contour_point_list;
			visualization_msgs::msg::Marker marker_cpts = createContourFrameMarker(contourpts,node);
			marker_cpts.id = objs[i].id;
			marker_cpts.ns = label + "_cpts";
			marker_cpts.header.frame_id = objList -> header.frame_id;
			cpts_marker_array.markers.push_back(marker_cpts);
		}

		marker_array.markers.push_back(marker_box);
		marker_array.markers.push_back(marker_label);
		
	}

	object_vis_pub->publish(marker_array);
	objectcpts_vis_pub->publish(cpts_marker_array);
}

//======================================================================

int main(int argc, char** argv)
{

	rclcpp::init(argc, argv);
	node=rclcpp::Node::make_shared("obj_visualization");
	// ros::NodeHandle np("~");

	std::string ibeo_object_topic; 

	node->declare_parameter("ibeo_object_topic","/object_tracker/ibeo_objects");
	node->get_parameter_or<std::string>("ibeo_object_topic",ibeo_object_topic,"/ibeo_8l_client/ibeo_objects");
	// ibeo_object_topic="/ibeo_objects";

	// RCLCPP_ERROR(node->get_logger(),ibeo_object_topic);
	// object_vis_pub = nh.advertise<visualization_msgs::MarkerArray> ("ibeo_objects_vis", 10);
	// objectcpts_vis_pub = nh.advertise<visualization_msgs::MarkerArray> ("ibeo_objects_vis_cpts", 10);

	object_vis_pub=node->create_publisher<visualization_msgs::msg::MarkerArray>("ibeo_objects_vis",10);
	objectcpts_vis_pub=node->create_publisher<visualization_msgs::msg::MarkerArray>("ibeo_objects_vis_cpts",10);
	// ros::Subscriber obj_sub = nh.subscribe(ibeo_object_topic, 10, visualCallback);
	auto obj_sub=node->create_subscription<ibeo_8l_msgs::msg::ObjectListLuxRos>(ibeo_object_topic,100,visualCallback);
	std::stringstream topic_notification;
	topic_notification<<"Subscribing to: "<<ibeo_object_topic;
	RCLCPP_ERROR(node->get_logger(),topic_notification.str());
	//ros::spin();
	rclcpp::spin(node);
	//RCLCPP_WARN(node->get_logger(),"Hi\n");
	exit(0);
}