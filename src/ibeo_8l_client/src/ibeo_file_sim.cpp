//======================================================================
/*! \file IbeoFileSim.cpp
 * Reading IDC files and publish data through time.
 * The publish message type has changed from PCL to sensor_msgs/msg/Pointcloud2
 *///-------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <ibeo_8l_msgs/msg/object_lux_ros.hpp>

#include <ibeosdk/ecu.hpp>
#include <ibeosdk/listener/DataListener.hpp>
#include <ibeosdk/devices/IdcFile.hpp>


#include <math.h>
#include <iostream>
#include <cstdlib>
#include <string.h>
#include <sstream>//ROS2 RCLCPP_INFO对字符串支持极差，需要使用stringstream整合
#include <rclcpp/node.hpp>

//======================================================================

using namespace ibeosdk;

TimeConversion tc;

#define SENSOR_FRAME "vehicle"

//======================================================================

class FileTopicConverter : public ibeosdk::DataListener<ScanEcu>,
                    public ibeosdk::DataListener<ObjectListEcu>,
                    public ibeosdk::DataListener<VehicleStateBasicEcu>,
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

	
	rclcpp::Time s_time_ros;
	ibeosdk::NTPTime s_time_ibeo;
	bool firstscan;

public:
	FileTopicConverter()
	:Node("ibeo_file_sim")
	{
		log_pub=this->create_publisher<std_msgs::msg::String>("log",1000);
		cloud_pub_up=this->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_up", 2);
		cloud_pub_down=this->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_down", 2);
		object_pub=this->create_publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>("ibeo_objects",10);
		firstscan=true;
	}
	virtual ~FileTopicConverter() {}

	
	void initPublisher(){
		// log_pub=node->create_publisher<std_msgs::msg::String>("log",1000);
		// cloud_pub_up=node->create_publisher<pcl::PointCloud<pcl::PointXYZI>> ("ibeo_scan_up", 2);
		// cloud_pub_down=node->create_publisher<pcl::PointCloud<pcl::PointXYZI>> ("ibeo_scan_down", 2);
		// cloud_pub_up=node->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_up", 2);
		// cloud_pub_down=node->create_publisher<sensor_msgs::msg::PointCloud2> ("ibeo_scan_down", 2);
		// object_pub=node->create_publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>("ibeo_objects",10);

		// firstscan = true;
	}

public:
	//Data callback functions, will be called when receiving messages.
	void onData(const ScanEcu* const scan)
	{
		// std::cout<<"ON1"<<std::endl;
		//convert the ibeo msg to PCL
		pcl::PointCloud<pcl :: PointXYZI> cloud;
		cloud.height = 1;
		cloud.width = scan -> getNumberOfScanPoints();
		cloud.is_dense = false;
		cloud.points.resize (cloud.width * cloud.height);

		int i, pointnum;
		std :: vector<ScanPointEcu> scan_e = scan -> getScanPoints();
		pointnum = scan -> getNumberOfScanPoints();

		
		for( i = 0; i < pointnum; i ++){
			cloud.points[i].x = scan_e[i].getPositionX();
			cloud.points[i].y = scan_e[i].getPositionY();
			cloud.points[i].z = scan_e[i].getPositionZ();
			cloud.points[i].intensity=scan_e[i].getEchoPulseWidth();
		}
		cloud.header.frame_id = SENSOR_FRAME;
		
		std :: vector<ScannerInfo> info = scan -> getScannerInfos();

		sleepUntilLogTime(scan->getStartTimestamp());	//timing control, need further coding in order to synchronize with ins data
		
		if(info.size() > 0){	//to distinguish upper scan and lower scan by BeamTilt
			sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
			pcl::toROSMsg(cloud,*map_msg_ptr);
			if(info[0].getBeamTilt() < 0)
				cloud_pub_up->publish(*map_msg_ptr);
			else
				cloud_pub_down->publish(*map_msg_ptr);
		}
		
		std::stringstream input_stream;
		input_stream<<scan->getSerializedSize()<<" Bytes ScanEcu recieved: # "<<scan->getScanNumber()<<" #Pts: "<<scan->getNumberOfScanPoints()<<" ScanStart "<<tc.toString(scan->getStartTimestamp().toPtime(), 3);
		
		RCLCPP_INFO(node_logger,input_stream.str());
	}

	void onData(const ObjectListEcu* const objList){
		//ROS_INFO_STREAM("GET OBJECTLISTECU\n");
		std::vector< ibeosdk :: ObjectEcu > objs = objList -> getObjects();
		int i, j, n = objList -> getNumberOfObjects();
		ibeosdk :: Point2dFloat p;

		ibeo_8l_msgs :: msg :: ObjectListLuxRos obj_list;
		obj_list.number_of_objects = n;
		obj_list.header.stamp = this->get_clock()->now();
		obj_list.header.frame_id = SENSOR_FRAME;

		
		for(i = 0; i < n; i++){

			ibeo_8l_msgs::msg::ObjectLuxRos obj_ros;

			obj_ros.id = objs[i].getObjectId();
			obj_ros.age = objs[i].getObjectAge();
			obj_ros.timestamp = this->get_clock()->now().nanoseconds();
			//TODO: Is it correct?

			obj_ros.classification = objs[i].getClassification();
			obj_ros.classification_certainty = objs[i].getClassificationCertainty();
			obj_ros.classification_age = objs[i].getClassificationAge();

			p = objs[i].getBoundingBoxCenter();
			obj_ros.bounding_box_center.x = p.getX();
			obj_ros.bounding_box_center.y = p.getY();
			p = objs[i].getBoundingBoxSize();
			obj_ros.bounding_box_size.x = p.getX();
			obj_ros.bounding_box_size.y = p.getY();

			p = objs[i].getObjectBoxCenter();
			obj_ros.object_box_center.x = p.getX();
			obj_ros.object_box_center.y = p.getY();
			p = objs[i].getObjectBoxSize();
			obj_ros.object_box_size.x = p.getX();
			obj_ros.object_box_size.y = p.getY();
			
			obj_ros.object_box_orientation = objs[i].getObjectBoxOrientation();

			p = objs[i].getRelativeVelocity();
			obj_ros.relative_velocity.x = p.getX();
			obj_ros.relative_velocity.y = p.getY();

			p = objs[i].getAbsoluteVelocity();
			obj_ros.absolute_velocity.x = p.getX();
			obj_ros.absolute_velocity.y = p.getY();
			p = objs[i].getAbsoluteVelocitySigma();
			obj_ros.absolute_velocity_sigma.x = p.getX();
			obj_ros.absolute_velocity_sigma.y = p.getY();

			obj_ros.number_of_contour_points = objs[i].getNumberOfContourPoints();

			//obj_ros.closest_point_index = objs[i].getIndexOfClosestPoint();
			//didn't understand what does the index means.

			ibeo_8l_msgs :: msg:: Point2Df p_ros;
			std :: vector < ibeosdk :: Point2dFloat > cpts = objs[i].getContourPoints();
			obj_ros.contour_point_list.reserve(obj_ros.number_of_contour_points);
			
			for(j = 0; j < obj_ros.number_of_contour_points; j ++){
				p_ros.x = cpts[j].getX(); p_ros.y = cpts[j].getY();
				obj_ros.contour_point_list.push_back(p_ros);
			}

			obj_list.object_list.push_back(obj_ros);
		}

		object_pub->publish(obj_list);
		std::stringstream input_stream;
		input_stream<<objList->getSerializedSize()<<" Bytes ObjectListEcu recieved: # "<<objList->getNumberOfObjects()<<" Time stamp: "<<tc.toString(objList->getTimestamp().toPtime(), 3);
		RCLCPP_INFO(node_logger,input_stream.str());
	}

	void onData(const VehicleStateBasicEcu* const vsb)
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

  private:
  	void sleepUntilLogTime(ibeosdk :: NTPTime target_time_ibeo){
		
		if(firstscan){
			firstscan = false;
			s_time_ros = this->get_clock()->now();
			s_time_ibeo = target_time_ibeo;
		}
		ibeosdk::NTPTime target_duration_ibeo = target_time_ibeo - s_time_ibeo;
		unsigned int s, ns;
		target_duration_ibeo.getTime_s_us(s, ns);
		ns *= 1000;
		rclcpp::Duration target_duration_ros=rclcpp::Duration((int)s, (int)ns);

		rclcpp::Duration ds = target_duration_ros - (this->get_clock()->now() - s_time_ros);
		if (ds.seconds()>0) 
		{
			rclcpp::sleep_for(std::chrono::nanoseconds(ds.nanoseconds()));
		}
		
	}

}; // AllListener


//======================================================================

int main(int argc, char** argv)
{

	rclcpp::init(argc, argv);
	// auto nh=rclcpp::Node::make_shared("ibeo_file_sim");
	auto converter=std::make_shared<FileTopicConverter>();
	//FileTopicConverter converter;
	std::string filename;
	//converter.get_parameter<std::string>("idc_file",filename);
	filename="/home/tom/ibeo_ws/src/ibeo_8l_client/record/20190320-085450.idc";
	
	
	if(filename == ""){
		RCLCPP_WARN(converter->get_logger(),"No IDC File Specified!\n");
		exit(0);
	}
	
	//Initialize the log file manager, which is essential for File Simulation
	const off_t maxLogFileSize = 1000000;
	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();
	
	if(rclcpp::ok()){
		
		IdcFile file;
		file.open(filename);

		if (file.isOpen() && rclcpp::ok()) {
			FileTopicConverter* converter_ptr=converter.get();
			file.registerListener(converter_ptr);

			const DataBlock* db = NULL;

			while (file.isGood() && rclcpp::ok()) {
				db = file.getNextDataBlock();

				//ROS_INFO_STREAM("GET\n");
				if (db == NULL) {
					continue; // might be eof or unknown file type

				}

				file.notifyListeners(db);
				//ROS_INFO_STREAM("INFOMED\n");

			}
		}
		else {
			RCLCPP_ERROR(converter->get_logger(),"File not readable / non-exist.");
		}
	}

	exit(0);
}