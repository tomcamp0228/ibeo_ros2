//======================================================================
/*! \file ibeo_live.cpp
 * Connect to a Ibeo 8l LIDAR and publish data to topics.
 * The publish message type has changed from PCL to sensor_msgs/msg/Pointcloud2
 *///-------------------------------------------------------------------
//TODO:import intensity function.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>


#include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <ibeo_8l_msgs/msg/object_lux_ros.hpp>


#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>
#include <ibeosdk/listener/DataListener.hpp>
// #include <ibeosdk/datablocks/commands/CommandEcuAppBaseStatus.hpp>
// #include <ibeosdk/datablocks/commands/ReplyEcuAppBaseStatus.hpp>
// #include <ibeosdk/datablocks/commands/CommandEcuAppBaseCtrl.hpp>
// #include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>

#include <iostream>
#include <cstdlib>
#include <string.h>
#include <math.h>
#include <sstream>//ROS2 RCLCPP_INFO对字符串支持极差，需要使用stringstream整合
#include <rclcpp/node.hpp>

//======================================================================

using namespace ibeosdk;

TimeConversion tc;

#define SENSOR_FRAME "vehicle"

//======================================================================

class IbeoLuxListener : public ibeosdk::DataListener<ScanLux>,
                    public ibeosdk::DataListener<ObjectListLux>,
                    public ibeosdk::DataListener<VehicleStateBasicLux>,
                    public ibeosdk::DataListener<LogMessageError>,
                    public ibeosdk::DataListener<LogMessageWarning>,
                    public ibeosdk::DataListener<LogMessageNote>,
                    public ibeosdk::DataListener<LogMessageDebug>,
					public rclcpp::Node
					{

private:
	// rclcpp::Node::SharedPtr node;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub;
	// rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>>::SharedPtr cloud_pub_up;
	// rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>>::SharedPtr cloud_pub_down;
	rclcpp::Publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>::SharedPtr object_pub;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_up;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_down;
	rclcpp::Logger node_logger=this->get_logger();

public:
	IbeoLuxListener()
	:Node("ibeo_live")
	{
		log_pub=this->create_publisher<std_msgs::msg::String>("log",1000);
		// cloud_pub_up=node->create_publisher<pcl::PointCloud<pcl::PointXYZI>> ("ibeo_scan_up", 2);
		// cloud_pub_down=node->create_publisher<pcl::PointCloud<pcl::PointXYZI>> ("ibeo_scan_down", 2);
		cloud_pub_up=this->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_up", 2);
		cloud_pub_down=this->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_down", 2);
		object_pub=this->create_publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>("ibeo_objects",10);
	}
	virtual ~IbeoLuxListener() {}
	//ibeosdk::IbeoDeviceBase::

public:
	//Data callback functions

	//convert the ibeo scan msg to PCL
	void onData(const ScanLux* const scan)
	{

		//get the scan angles
		int i, point_num, angletick_r;
		uint8_t mirror_flag;
		double angle, dist, vertical_angle, base_vertical_angle;
		std :: vector<ScanPointLux> scan_e = scan -> getScanPoints();
		angletick_r = scan -> getAngleTicksPerRotation();
		point_num = scan -> getNumberOfScanPoints();

		//initialize the PCL point cloud
		pcl::PointCloud<pcl :: PointXYZI> cloud;
		cloud.height = 1;
		cloud.width = point_num;
		cloud.is_dense = false;
		cloud.points.resize (cloud.width * cloud.height);

		//distinguish whether the data is from top 4 layer or bottom 4 layer
		mirror_flag = scan ->getFlags() >> 10 & 0x01;
		if(mirror_flag == 0x01)
			base_vertical_angle = 0.4;
		else
			base_vertical_angle = -2.8;

		//fill the point cloud
		for( i = 0; i < point_num; i ++){

			angle = 2 * M_PI * (double)scan_e[i].getHorizontalAngle() / angletick_r;
			dist = scan_e[i].getDistance() / 100.0;
			vertical_angle = ((int)scan_e[i].getLayer() * 0.8 + base_vertical_angle) /180.0 * M_PI;

			cloud.points[i].x = cos(angle) * cos(vertical_angle) * dist;
			cloud.points[i].y = sin(angle) * cos(vertical_angle) * dist;
			cloud.points[i].z = sin(vertical_angle) * dist;
		}
		cloud.header.frame_id = SENSOR_FRAME;
		sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(cloud,*map_msg_ptr);
		//publish the PCL msg
		if(mirror_flag == 0x01)
			cloud_pub_up->publish(*map_msg_ptr);
		else
		 	cloud_pub_down->publish(*map_msg_ptr);
		std::stringstream input_stream;
		input_stream<<scan->getSerializedSize()<<" Bytes ScanEcu recieved: # "<<scan->getScanNumber()<<" #Pts: "<<scan->getNumberOfScanPoints()<<" ScanStart "<<tc.toString(scan->getStartTimestamp().toPtime(), 3);
		
		RCLCPP_INFO(node_logger,input_stream.str());
		// RCLCPP_INFO(node->get_logger(),"%s Bytes ScanEcu recieved: # %d #Pts: %d ScanStart %s"
		// 								,scan->getSerializedSize(),scan->getScanNumber(),scan->getNumberOfScanPoints(),tc.toString(scan->getStartTimestamp().toPtime(), 3));
	}

	//convert the ibeo object msg to ros msg
	void onData(const ObjectListLux* const objList){
		std::vector< ibeosdk :: ObjectLux > objs = objList -> getObjects();
		int i, j, n = objList -> getNumberOfObjects();

		ibeo_8l_msgs :: msg :: ObjectListLuxRos obj_list;
		obj_list.number_of_objects = n;
		obj_list.header.stamp = this->get_clock()->now();
		obj_list.header.frame_id = SENSOR_FRAME;
		
		for(i = 0; i < n; i++){

			ibeo_8l_msgs::msg :: ObjectLuxRos obj_ros;

			obj_ros.id = objs[i].getObjectId();
			obj_ros.age = objs[i].getObjectAge();
			obj_ros.timestamp = float(this->get_clock()->now().seconds());

			obj_ros.classification = objs[i].getClassification();
			obj_ros.classification_certainty = objs[i].getClassificationCertainty();
			obj_ros.classification_age = objs[i].getClassificationAge();
			obj_ros.prediction_age = objs[i].getPredictionAge();

			ibeosdk :: Point2d p;
			ibeosdk :: PointSigma2d p_sig;

			p = objs[i].getBoundingBoxCenter();
			obj_ros.bounding_box_center.x = p.getX() / 100.0;
			obj_ros.bounding_box_center.y = p.getY() / 100.0;
			obj_ros.bounding_box_size.x = objs[i].getBoundingBoxLength() / 100.0;
			obj_ros.bounding_box_size.y = objs[i].getBoundingBoxWidth() / 100.0;

			p = objs[i].getReferencePoint();
			obj_ros.reference_point.x = p.getX() / 100.0;
			obj_ros.reference_point.y = p.getY() / 100.0;
			p_sig = objs[i].getReferencePointSigma();
			obj_ros.reference_point_sigma.x = p_sig.getX() / 100.0;
			obj_ros.reference_point_sigma.y = p_sig.getY() / 100.0;

			p = objs[i].getObjectBoxCenter();
			obj_ros.object_box_center.x = p.getX() / 100.0;
			obj_ros.object_box_center.y = p.getY() / 100.0;

			obj_ros.object_box_size.x = objs[i].getObjectBoxSizeX() / 100.0;
			obj_ros.object_box_size.y = objs[i].getObjectBoxSizeY() / 100.0;
			
			obj_ros.object_box_orientation = objs[i].getObjectBoxOrientation() / 100.0 * (M_PI / 180.0);

			p = objs[i].getRelativeVelocity();
			obj_ros.relative_velocity.x = p.getX() / 100.0;
			obj_ros.relative_velocity.y = p.getY() / 100.0;

			p = objs[i].getAbsoluteVelocity();
			obj_ros.absolute_velocity.x = p.getX() / 100.0;
			obj_ros.absolute_velocity.y = p.getY() / 100.0;
			obj_ros.absolute_velocity_sigma.x = objs[i].getAbsoluteVelocitySigmaX() / 100.0;
			obj_ros.absolute_velocity_sigma.y = objs[i].getAbsoluteVelocitySigmaY() / 100.0;

			p = objs[i].getClosestPoint();
			obj_ros.closest_point.x = p.getX() / 100.0;
			obj_ros.closest_point.y = p.getY() / 100.0;

			//contour points
			obj_ros.number_of_contour_points = objs[i].getNumberOfContourPoints();
			ibeo_8l_msgs :: msg :: Point2Df p_ros;
			std :: vector < ibeosdk :: Point2d > cpts = objs[i].getContourPoints();
			obj_ros.contour_point_list.reserve(obj_ros.number_of_contour_points);
			for(j = 0; j < obj_ros.number_of_contour_points; j ++){
				p_ros.x = cpts[j].getX() / 100.0;
				p_ros.y = cpts[j].getY() / 100.0;
				obj_ros.contour_point_list.push_back(p_ros);
			}

			obj_list.object_list.push_back(obj_ros);
		}

		object_pub->publish(obj_list);
		std::stringstream input_stream;
		input_stream<<objList->getSerializedSize()<<" Bytes ObjectListEcu recieved: # "<<objList->getNumberOfObjects()<<" Time stamp: "<<tc.toString(objList->getScanStartTimestamp().toPtime(), 3);
		RCLCPP_INFO(node_logger,input_stream.str());
	}

	void onData(const VehicleStateBasicLux* const vsb)
	{
		std::stringstream input_stream;
		input_stream<< vsb->getSerializedSize()<<" Bytes  VSB (ECU) received: time: "<<tc.toString(vsb->getTimestamp().toPtime());
		RCLCPP_INFO(node_logger,input_stream.str());
	}

	void onData(const LogMessageError* const logMsg)
	{
		std::stringstream input_stream;
		input_stream<<logMsg->getSerializedSize()<<" Bytes LogMessage (Error) received: time: "<<logMsg->getTraceLevel()<<" : "<<logMsg->getMessage();
		RCLCPP_ERROR(node_logger,input_stream.str());
	}
	void onData(const LogMessageWarning* const logMsg)
	{
		std::stringstream input_stream;
		input_stream<<logMsg->getSerializedSize()<<" Bytes LogMessage (Warning) received: time: "<<logMsg->getTraceLevel()<<" : "<<logMsg->getMessage();
		RCLCPP_WARN(node_logger,input_stream.str());
	}
	void onData(const LogMessageNote* const logMsg)
	{
		std::stringstream input_stream;
		input_stream<<logMsg->getSerializedSize()<<" Bytes LogMessage (Note) received: time: "<<logMsg->getTraceLevel()<<" : "<<logMsg->getMessage();
		RCLCPP_INFO(node_logger,input_stream.str());
	}
	void onData(const LogMessageDebug* const logMsg)
	{
		std::stringstream input_stream;
		input_stream<<logMsg->getSerializedSize()<<" Bytes LogMessage (Debug) received: time: "<<logMsg->getTraceLevel()<<" : "<<logMsg->getMessage();
		RCLCPP_DEBUG(node_logger,input_stream.str());
	}
};


//======================================================================

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto luxListener=std::make_shared<IbeoLuxListener>();
	rclcpp::executors::SingleThreadedExecutor excutor;
	
	// auto nh=luxListener.make_shared();
	std::string ip;
	int port;
	// luxListener.get_parameter<std::string>("device_ip", ip);
	// luxListener.get_parameter<int>("device_port",port);
	ip="192.168.1.16";
	port=12002;

	if(ip == ""){
		RCLCPP_ERROR(luxListener->get_logger(),"Invalid IP!\n");
		exit(0);
	}

	RCLCPP_INFO(luxListener->get_logger(),"Connecting to ECU @ %s:%d", ip.c_str(), port);

	//Initialize the log file manager, which is essential
	const off_t maxLogFileSize = 1000000;
	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Info");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();

	

	const uint16_t format_port = getPort(ip, port);
	IbeoLux lux(ip, format_port);
	lux.setLogFileManager(&logFileManager);
	IbeoLuxListener* luxListener_ptr=luxListener.get();
	lux.registerListener(luxListener_ptr);
	RCLCPP_INFO(luxListener->get_logger(),"Trying to connect...");
	lux.getConnected();
	RCLCPP_INFO(luxListener->get_logger(),"Waiting to connect2...");
	//Wait for connection
	sleep(0.1);
	RCLCPP_INFO(luxListener->get_logger(),"Wakeup...");
	if(lux.isConnected()){
		RCLCPP_INFO(luxListener->get_logger(),"Connected. Receiving Packages.");
		rclcpp::spin_some(luxListener);
		
	}
	else
		{RCLCPP_ERROR(luxListener->get_logger(),"Target IP not responding / refuse connection!\n");
		exit(0);
		}
	exit(0);
}