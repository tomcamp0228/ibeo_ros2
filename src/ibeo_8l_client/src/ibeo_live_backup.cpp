//======================================================================
/*! \file ibeo_visualization.cpp
 *
 * \copydoc Copyright
 * \author Kexin Zheng
 * \date April 17, 2019
 *
 * Use Object data in ibeo_8l_msgs to demonstrate visualization
 *///-------------------------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>
#include "pcl_ros/point_cloud.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include <ibeo_8l_msgs/ObjectLuxRos.h>
#include <ibeo_8l_msgs/ObjectListLuxRos.h>

#include <iostream>
#include <cstdlib>
#include <math.h>


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
                    public ibeosdk::DataListener<LogMessageDebug> {

private:
	ros::NodeHandle *nh_;
	ros::Publisher log_pub;
	ros::Publisher cloud_pub_up;
	ros::Publisher cloud_pub_down;
	ros::Publisher objectcpts_vis_pub;
	ros::Publisher object_vis_pub;
	ros::Publisher object_pub;

public:
	virtual ~IbeoLuxListener() {}
	void setNodeHandle(const ros::NodeHandle* nh){
		nh_ = (ros::NodeHandle*)nh;
		initPublisher();
	}
	
	void initPublisher(){
		log_pub = nh_ -> advertise<std_msgs::String>("log", 1000);
		cloud_pub_up = nh_ -> advertise<pcl::PointCloud<pcl::PointXYZ> > ("ibeo_scan_up", 2);
		cloud_pub_down = nh_ -> advertise<pcl::PointCloud<pcl::PointXYZ> > ("ibeo_scan_down", 2);
		object_vis_pub = nh_ -> advertise<visualization_msgs::MarkerArray> ("ibeo_objects_vis", 10);
		objectcpts_vis_pub = nh_ -> advertise<visualization_msgs::MarkerArray> ("ibeo_objects_vis_cpts", 10);
		object_pub = nh_ -> advertise<ibeo_8l_msgs::ObjectListLuxRos> ("ibeo_objects", 10);
	}

public:
	//Data callback functions, will be called when receiving messages.
	void onData(const ScanLux* const scan)
	{

		//convert the ibeo msg to PCL
		pcl::PointCloud<pcl :: PointXYZ> cloud;
		cloud.height = 1;
		cloud.width = scan -> getNumberOfScanPoints();
		cloud.is_dense = false;
		cloud.points.resize (cloud.width * cloud.height);


		//get the scan angles
		int i, pointnum, angletick_r;
		uint8_t mirror_flag;
		double angle, dist, vertical_angle, base_vertical_angle;
		std :: vector<ScanPointLux> scan_e = scan -> getScanPoints();
		angletick_r = scan -> getAngleTicksPerRotation();
		pointnum = scan -> getNumberOfScanPoints();
		mirror_flag = scan ->getFlags() >> 10 & 0x01;
		if(mirror_flag == 0x01) base_vertical_angle = 0.4;
		else base_vertical_angle = -2.8;
		


		for( i = 0; i < pointnum; i ++){
			angle = 2 * M_PI * (double)scan_e[i].getHorizontalAngle() / angletick_r;
			dist = scan_e[i].getDistance() / 100.0;
			vertical_angle = ((int)scan_e[i].getLayer() * 0.8 + base_vertical_angle) /180.0 * M_PI;

			cloud.points[i].x = cos(angle) * dist;
			cloud.points[i].y = sin(angle) * dist;
			cloud.points[i].z = tan(vertical_angle) * dist;
		}
		cloud.header.frame_id = SENSOR_FRAME;


		if(mirror_flag == 0x01)
			cloud_pub_up.publish(cloud);
		else
		 	cloud_pub_down.publish(cloud);

		std :: stringstream sslog;
		sslog << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "ScanEcu received: # " << scan->getScanNumber()
				<< "  #Pts: " << scan->getNumberOfScanPoints()
				<< "  ScanStart: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
				<< std::endl;
		ROS_INFO_STREAM(sslog.str());
	}

	void onData(const ObjectListLux* const objList){

		std::vector< ibeosdk :: ObjectLux > objs = objList -> getObjects();

		int i, n = objList -> getNumberOfObjects();
		ibeosdk :: Point2d p1, p2;

		//create the array-type msg for markers and object info
		visualization_msgs :: MarkerArray marker_array;
		visualization_msgs :: MarkerArray cpts_marker_array;
		ibeo_8l_msgs :: ObjectListLuxRos obj_list;
		obj_list.number_of_objects = n;
		obj_list.header.stamp = obj_list.scan_start_timestamp = ros::Time::now();
		obj_list.header.frame_id = SENSOR_FRAME;

		for(i = 0; i < n; i++){

			std::vector< ibeosdk::Point2d > contourpts = objs[i].getContourPoints();
			p1 = objs[i].getObjectBoxCenter();

			//create the object box vislization msg
			visualization_msgs::Marker marker_box = createWireframeMarker(p1.getX() / 100.0, p1.getY() / 100.0,
													objs[i].getObjectBoxSizeX() / 100.0, objs[i].getObjectBoxSizeY() / 100.0, 0.75);
			//create contour points vislization msg
			visualization_msgs::Marker marker_cpts = createContourFrameMarker(contourpts);
			marker_cpts.id = objs[i].getObjectId();

			marker_box.id = objs[i].getObjectId();
			tf::Quaternion qn = tf::createQuaternionFromYaw(objs[i].getObjectBoxOrientation());
			marker_box.pose.orientation.x = qn.x();
			marker_box.pose.orientation.y = qn.y();
			marker_box.pose.orientation.z = qn.z();
			marker_box.pose.orientation.w = qn.w();

			marker_box.lifetime = ros::Duration(0.5);
			marker_box.color.a = 0.5;
			marker_box.color.r = marker_box.color.g = marker_box.color.b = 1.0;
    		marker_box.frame_locked = false;
			marker_box.scale.x = 0.05;
			marker_box.scale.y = 0.05;
			marker_box.scale.z = 0.05;
			marker_box.type = visualization_msgs::Marker::LINE_LIST;
			marker_box.action = visualization_msgs::Marker::MODIFY;
			marker_box.header.stamp = ros::Time::now();
			marker_box.header.frame_id = SENSOR_FRAME;

			//fill the label text with classification
			std::string label;
			switch (objs[i].getClassification())
			{
				case luxObjectClass::LuxObjectClass_Unclassified:
					label = "Unclassified";
					// Unclassified - white
					break;
				case luxObjectClass::LuxObjectClass_UnknownSmall:
					label = "Unknown Small";
					// Unknown small - blue
					marker_box.color.r = marker_box.color.g = 0;
					break;
				case luxObjectClass::LuxObjectClass_UnknownBig:
					label = "Unknown Big";
					// Unknown big - dark blue
					marker_box.color.r = marker_box.color.g = 0;
					marker_box.color.b = 0.5;
					break;
				case luxObjectClass::LuxObjectClass_Pedestrian:
					label = "Pedestrian";
					// Pedestrian - red
					marker_box.color.g = marker_box.color.b = 0;
					break;
				case luxObjectClass::LuxObjectClass_Bike:
					label = "Bike";
					// Bike - dark red
					marker_box.color.g = marker_box.color.b = 0;
					marker_box.color.r = 0.5;
					break;
				case luxObjectClass::LuxObjectClass_Bicycle:
					label = "Bicycle";
					// Bicycle - dark red
					marker_box.color.g = marker_box.color.b = 0;
					marker_box.color.r = 0.5;
					break;
				case luxObjectClass::LuxObjectClass_Car:
					label = "Car";
					// Car - green
					marker_box.color.b = marker_box.color.r = 0;
					break;
				case luxObjectClass::LuxObjectClass_Truck:
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
			marker_cpts.ns = label + "_cpts";

			//create label visualization msg to label the object
			visualization_msgs::Marker marker_label;
			marker_label.id  = objs[i].getObjectId() + 1000;	//to avoid conflict of ID
			marker_label.ns = label;
			marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker_label.action = visualization_msgs::Marker::MODIFY;
			marker_label.pose.position.x = p1.getX() / 100.0;
			marker_label.pose.position.y = p1.getY() / 100.0;
			marker_label.pose.position.z = 0.5;
			marker_label.text = label + ", id: " + boost::to_string(objs[i].getObjectId());
			marker_label.scale.x = 0.1;
			marker_label.scale.y = 0.1;
			marker_label.scale.z = 0.5;
			marker_label.lifetime = marker_box.lifetime;
			marker_label.color.r = marker_label.color.g = marker_label.color.b = 1;
			marker_label.color.a = 0.5;
			marker_label.header.stamp = ros::Time::now();
			marker_label.header.frame_id = SENSOR_FRAME;

			marker_array.markers.push_back(marker_box);
			marker_array.markers.push_back(marker_label);

			cpts_marker_array.markers.push_back(marker_cpts);

			//Create the object ros msg with full info
			// (i.e. convert ibeo object to ros message)
			ibeo_8l_msgs::ObjectLuxRos obj_ros;
			ObjectLuxRosParser(obj_ros, objs[i]);
			obj_list.object_list.push_back(obj_ros);
		}

		object_vis_pub.publish(marker_array);
		objectcpts_vis_pub.publish(cpts_marker_array);
		object_pub.publish(obj_list);
		
		std :: stringstream sslog;
		sslog << std::setw(5) << objList->getSerializedSize() << " Bytes  "
		<< "ObjectListEcu received: # " << objList->getNumberOfObjects()
		<< "Time stamp: " << tc.toString(objList -> getScanStartTimestamp().toPtime(), 3) << std::endl;
		ROS_INFO_STREAM(sslog.str());

	}

	void onData(const VehicleStateBasicLux* const vsb)
	{
		std :: stringstream sslog;
		sslog << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				<< "VSB (ECU) received: time: " << tc.toString(vsb->getTimestamp().toPtime())
				<< std::endl;
		ROS_INFO_STREAM(sslog.str());
	}

	void onData(const LogMessageError* const logMsg)
	{
		std :: stringstream sse;
		sse << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
		ROS_ERROR_STREAM(sse.str());
	}
	void onData(const LogMessageWarning* const logMsg)
	{
		std :: stringstream ssw;
		ssw << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
		ROS_WARN_STREAM(ssw.str());
	}
	void onData(const LogMessageNote* const logMsg)
	{
		std :: stringstream ssn;
		ssn << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
		ROS_INFO_STREAM(ssn.str());
	}
	void onData(const LogMessageDebug* const logMsg)
	{
		std :: stringstream ssd;
		ssd << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
		ROS_DEBUG_STREAM(ssd.str());
	}

  private:

	visualization_msgs::Marker createWireframeMarker(const float& center_x,	const float& center_y,
													float size_x,	float size_y,	const float& size_z){
	
		visualization_msgs::Marker box;
		box.pose.position.x = center_x;
		box.pose.position.y = center_y;
		geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

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

	visualization_msgs::Marker createContourFrameMarker(std::vector<ibeosdk::Point2d>& pts){

		visualization_msgs::Marker marker;
		marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.lifetime = ros::Duration(0.5);
		marker.color.a = 0.5;
		marker.color.r = 1.0; marker.color.g = marker.color.b = 0;
    	marker.frame_locked = false;
		marker.scale.x = 0.05;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = SENSOR_FRAME;

		marker.points.reserve(pts.size());
		int i;
		geometry_msgs::Point p1, p2;

		p1.x = p2.x = pts[0].getX() / 100.0;
		p1.y = p2.y = pts[0].getY() / 100.0;
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
	
	void ObjectLuxRosParser(ibeo_8l_msgs :: ObjectLuxRos& objros, ibeosdk :: ObjectLux& obj){

		objros.id = obj.getObjectId();
		objros.age = obj.getObjectAge();
		objros.timestamp = ros :: Time :: now();

		objros.classification = obj.getClassification();
		objros.classification_certainty = obj.getClassificationCertainty();
		objros.classification_age = obj.getClassificationAge();
		objros.prediction_age = obj.getPredictionAge();

		ibeosdk :: Point2d p;
		ibeosdk :: PointSigma2d p_sig;

		p = obj.getBoundingBoxCenter();
		objros.bounding_box_center.x = p.getX();
		objros.bounding_box_center.y = p.getY();
		objros.bounding_box_size.x = obj.getBoundingBoxLength();
		objros.bounding_box_size.y = obj.getBoundingBoxWidth();

		p = obj.getReferencePoint();
		objros.reference_point.x = p.getX();
		objros.reference_point.y = p.getY();
		p_sig = obj.getReferencePointSigma();
		objros.reference_point_sigma.x = p_sig.getX();
		objros.reference_point_sigma.y = p_sig.getY();

		p = obj.getObjectBoxCenter();
		objros.object_box_center.x = p.getX();
		objros.object_box_center.y = p.getY();

		objros.object_box_size.x = obj.getObjectBoxSizeX();
		objros.object_box_size.y = obj.getObjectBoxSizeY();
		
		objros.object_box_orientation = obj.getObjectBoxOrientation();

		p = obj.getRelativeVelocity();
		objros.relative_velocity.x = p.getX();
		objros.relative_velocity.y = p.getY();

		p = obj.getAbsoluteVelocity();
		objros.absolute_velocity.x = p.getX();
		objros.absolute_velocity.y = p.getY();
		objros.absolute_velocity_sigma.x = obj.getAbsoluteVelocitySigmaX();
		objros.absolute_velocity_sigma.y = obj.getAbsoluteVelocitySigmaY();

		p = obj.getClosestPoint();
		objros.closest_point.x = p.getX();
		objros.closest_point.y = p.getY();

		//contour points
		objros.number_of_contour_points = obj.getNumberOfContourPoints();
		int i;
		ibeo_8l_msgs :: Point2Df p_ros;
		std :: vector < ibeosdk :: Point2d > cpts = obj.getContourPoints();
		objros.contour_point_list.reserve(objros.number_of_contour_points);

		for(i = 0; i < objros.number_of_contour_points; i ++){
			p_ros.x = cpts[i].getX(); p_ros.y = cpts[i].getY();
			objros.contour_point_list.push_back(p_ros);
		}
		return;
	}

};


//======================================================================

int main(int argc, char** argv)
{

	ros::init(argc, argv, "ibeo_live");
	ros::NodeHandle nh;
	ros::NodeHandle np("~");

	std::string ip;
	int port;
	np.param<std::string>("device_ip", ip, "");
	np.param<int>("device_port", port, 12002);

	if(ip == ""){
		ROS_ERROR_STREAM("Invalid IP!\n");
		exit(0);
	}

	ROS_INFO("Connecting to ECU @ %s:%d", ip.c_str(), port);

	//Initialize the log file manager, which is essential
	const off_t maxLogFileSize = 1000000;
	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Info");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();

	IbeoLuxListener luxListener;
	luxListener.setNodeHandle(&nh);

	const uint16_t format_port = getPort(ip, port);
	IbeoLux lux(ip, format_port);
	lux.setLogFileManager(&logFileManager);

	lux.registerListener(&luxListener);
	lux.getConnected();

	//Wait for connection
	sleep(1);

	if(lux.isConnected()){
		ROS_INFO("Connected. Receiving Packages.");
		ros::waitForShutdown();
	}
	else
		ROS_ERROR_STREAM("Target IP not responding / refuse connection!\n");

	exit(0);
}