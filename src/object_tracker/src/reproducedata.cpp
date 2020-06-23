#include <visualization_msgs/msg/marker_array.hpp>
#include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include <stdio.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <object_tracker/car_tracker_kalman.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>

uint seq = 0;
// double realyaw[1000];
// ros::Publisher *objpub;
ros::Publisher *carpub;
ros::Publisher *trackpub;
tf::TransformBroadcaster *tf_pub;
ObjectTracker::CarKF Kal(0.1, 5);
ObjectTracker::CarKF Self(0.1, 0.5);
nav_msgs::Odometry currentOdom;

visualization_msgs::Marker createWireframeMarker(const float& center_x,	const float& center_y, const float& center_z,
                                                float size_x,	float size_y, const float& size_z);

// void Callback1(const visualization_msgs::MarkerArray::ConstPtr& msg){

//     int i;
//     visualization_msgs::MarkerArray nmsg;
//     for(i = 0; i < msg -> markers.size(); i ++){
//         if(msg -> markers[i].id < 1000){
//             tf::Quaternion q = tf::createQuaternionFromYaw(realyaw[msg -> markers[i].id]);
//             visualization_msgs::Marker nmarker;
//             nmarker.pose.orientation.x = q.getX();
//             nmarker.pose.orientation.y = q.getY();
//             nmarker.pose.orientation.z = q.getZ();
//             nmarker.pose.orientation.w = q.getW();

//             nmarker.header = msg ->markers[i].header;
//             nmarker.id = msg->markers[i].id;
//             nmarker.lifetime = msg->markers[i].lifetime;
//             nmarker.color = msg->markers[i].color;
//             nmarker.scale = msg->markers[i].scale;
//             nmarker.type = msg->markers[i].type;
//             nmarker.action = msg->markers[i].action;
//             nmarker.frame_locked = msg->markers[i].frame_locked;
//             nmarker.pose.position = msg->markers[i].pose.position;
//             nmarker.points = msg->markers[i].points;
//             nmsg.markers.push_back(nmarker);
//         }
//     }
//     objpub -> publish(nmsg);
//     return;
// }

void Callback2(const ibeo_8l_msgs::ObjectListLuxRos::ConstPtr& msg){

    int i, carindex;
    double tmpx, tmpy;
    ibeo_8l_msgs::ObjectLuxRos car;
    ibeo_8l_msgs::ObjectListLuxRos surround;
    surround.number_of_objects = 0;
    
    for(i = 0; i < msg ->number_of_objects; i ++){
        // if(msg ->object_list[i].id < 1000){
            // realyaw[msg -> object_list[i].id] = msg -> object_list[i].object_box_orientation / 100.0 / 180.0 * M_PI;
            if(msg ->object_list[i].id == 135){
                carindex = i;
                car = msg ->object_list[i];
                car.object_box_center.x /= 100.0;
                car.object_box_center.y /= 100.0;
                car.object_box_size.x /= 100.0;
                car.object_box_size.y /= 100.0;
                break;
            }
        // }
    }
    for(i = 0; i < msg ->number_of_objects; i ++){
        tmpx = msg ->object_list[i].object_box_center.x / 100.0;
        tmpy = msg ->object_list[i].object_box_center.y / 100.0;
        if(pow(tmpx - car.object_box_center.x, 2) + pow(tmpy - car.object_box_center.y, 2) < 7.0){
            surround.object_list.push_back(msg ->object_list[i]);
            surround.number_of_objects ++;
        }
    }
    ROS_INFO("Point # %d \n", surround.number_of_objects);

    if(surround.number_of_objects == 0) return;
    
    double avgcount = 0.0;
    double value = 0.0;
    double posx = 0, posy = 0;
    double velx = 0, vely = 0;
    double avgyaw = 0;
    for(i = 0;i < surround.number_of_objects; i ++){
        value = sqrt(surround.object_list[i].object_box_size.x / 100.0 * surround.object_list[i].object_box_size.y / 100.0);
        avgcount += value;
        posx += surround.object_list[i].object_box_center.x / 100.0 * value;
        posy += surround.object_list[i].object_box_center.y / 100.0 * value;
        velx += surround.object_list[i].relative_velocity.x / 100.0 * value;
        vely += surround.object_list[i].relative_velocity.y / 100.0 * value;
        avgyaw +=surround.object_list[i].object_box_orientation / 100.0 / 180.0 * M_PI;
    }
    posx /= avgcount;
    posy /= avgcount;
    velx /= avgcount;
    vely /= avgcount;
    avgyaw /= avgcount;

    //convert to map coordinate
    tf::Quaternion vehicleq;
    vehicleq.setX(currentOdom.pose.pose.orientation.x);
    vehicleq.setY(currentOdom.pose.pose.orientation.y);
    vehicleq.setZ(currentOdom.pose.pose.orientation.z);
    vehicleq.setW(currentOdom.pose.pose.orientation.w);
    double vehicleyaw = tf::getYaw(vehicleq);
    if(isnan(vehicleyaw)) return;
    ROS_INFO("Vehicle Yaw:%lf\n", vehicleyaw);
    double mapposx, mapposy, mapvelx, mapvely;
    mapposx = posx * cos(vehicleyaw) - posy * sin(vehicleyaw) + currentOdom.pose.pose.position.x;
    mapposy = posx * sin(vehicleyaw) + posy * cos(vehicleyaw) + currentOdom.pose.pose.position.y;
    mapvelx = velx * cos(vehicleyaw) - vely * sin(vehicleyaw) + currentOdom.twist.twist.linear.x;
    mapvely = velx * sin(vehicleyaw) + vely * cos(vehicleyaw) + currentOdom.twist.twist.linear.y;
    avgyaw += vehicleyaw;
    if(avgyaw > 2*M_PI) avgyaw -= 2*M_PI;

    if(isnan(mapposx)||isnan(mapposy)||isnan(mapvelx)||isnan(mapvely)) return;
    Eigen::Vector4d m(mapposx, mapposy, mapvelx, mapvely);
    Kal.measurementUpdate(m, msg -> header.stamp);

    double yaw = atan(Kal.getVelY() / Kal.getVelX());
    ROS_INFO("VelX:%lf, VelY:%lf\n", Kal.getVelX(), Kal.getVelY());
    if(Kal.getVelX() < 0) yaw += M_PI;
    if(yaw < 0) yaw = yaw + (2 * M_PI);
    ROS_INFO("Yaw:%lf, Avgyaw:%lf\n", yaw, avgyaw);


    // double factor;
    // if(abs(avgyaw - vehicleyaw) > 0.7)
    //     factor = 0.1;
    // else
    //     factor = 0.8;
    // if(avgyaw > M_PI) avgyaw -= (2*M_PI);
    // if(yaw > M_PI) yaw -= (2*M_PI);
    // yaw = factor * avgyaw + (1 - factor) * yaw;
    // if(yaw < 0) yaw += 2*M_PI;

    //use yaw, Kalpos to build visualization marker
    visualization_msgs::Marker marker = createWireframeMarker(Kal.getPosX(), Kal.getPosY(), 1.0, 3.0, 1.5, 1);
    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    marker.header.frame_id = "map";
    marker.header.stamp = msg ->header.stamp;
    marker.id = 1;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 0.5;
    marker.color.r = 0; marker.color.g = 1; marker.color.b = 0;
    marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.frame_locked = false;
    // marker.pose.position.x = Kal.getPosX();
    // marker.pose.position.y = Kal.getPosY();
    
    //shift position of center
    double shiftx, shifty, shiftl;
    shiftx = Kal.getPosX() - currentOdom.pose.pose.position.x;
    shifty = Kal.getPosY() - currentOdom.pose.pose.position.y;
    avgcount = sqrt(pow(shiftx, 2) + pow(shifty, 2));
    shiftx /= avgcount;
    shifty /= avgcount;
    double relyaw;
    relyaw = atan(shifty / shiftx);
    if(shiftx < 0) relyaw += M_PI;
    if(relyaw < 0) relyaw = relyaw + (2 * M_PI);
    relyaw = relyaw - vehicleyaw;
    // if(relyaw < 0) relyaw += 2 * M_PI;
    // if(relyaw > M_PI) relyaw -= M_PI;
    ROS_INFO("relyaw:%lf\n", relyaw);
    shiftl = fabs(sin(relyaw)) * 0.75 + fabs(cos(relyaw)) * 1.5;
    ROS_INFO("Shiftl:%lf\n", shiftl);
    shiftx *= shiftl; shifty *= shiftl;
    marker.pose.position.x += shiftx;
    marker.pose.position.y += shifty;

    ibeo_8l_msgs::ObjectListLuxRos objlist;
    ibeo_8l_msgs::ObjectLuxRos trackedcar;
    trackedcar.id = 1;
    trackedcar.classification = ibeo_8l_msgs::ObjectLuxRos::CAR;
    trackedcar.object_box_center.x = marker.pose.position.x;
    trackedcar.object_box_center.y = marker.pose.position.y;
    trackedcar.object_box_orientation = yaw;
    trackedcar.object_box_size.x = 3.0;
    trackedcar.object_box_size.y = 1.5;
    trackedcar.absolute_velocity.x = Kal.getVelX();
    trackedcar.absolute_velocity.y = Kal.getVelY();
    trackedcar.timestamp = msg -> header.stamp;
    trackedcar.relative_velocity.x = velx;
    trackedcar.relative_velocity.y = vely;

    objlist.header = msg -> header;
    objlist.number_of_objects = 1;
    objlist.object_list.push_back(trackedcar);
    objlist.scan_start_timestamp = msg -> header.stamp;
    trackpub -> publish(objlist);
    

    visualization_msgs::Marker marker2;
    yaw = atan(mapvely / mapvelx);
    if(mapvely < 0) yaw += M_PI;
    yaw += M_PI;
    if(yaw > 2 * M_PI) yaw = yaw - 2 * M_PI;
    q = tf::createQuaternionFromYaw(yaw);
    marker2.pose.orientation.x = q.getX();
    marker2.pose.orientation.y = q.getY();
    marker2.pose.orientation.z = q.getZ();
    marker2.pose.orientation.w = q.getW();

    marker2.header.frame_id = "map";
    marker2.header.stamp = msg ->header.stamp;
    marker2.id = 2;
    marker2.lifetime = ros::Duration(0.5);
    marker2.color.a = 0.5;
    marker2.color.r = 1; marker2.color.g = 0; marker2.color.b = 0;
    marker2.scale.x = 2; marker2.scale.y = 0.3; marker2.scale.z = 0.3;
    marker2.type = visualization_msgs::Marker::CUBE;
    marker2.action = visualization_msgs::Marker::MODIFY;
    marker2.frame_locked = false;
    marker2.pose.position.x = mapposx;
    marker2.pose.position.y = mapposy;


    visualization_msgs::MarkerArray markera;
    markera.markers.push_back(marker);
    markera.markers.push_back(marker2);

    carpub -> publish(markera);

    return;
}

visualization_msgs::Marker createWireframeMarker(const float& center_x,	const float& center_y, const float& center_z,
                                                float size_x,	float size_y, const float& size_z){

    visualization_msgs::Marker box;
    box.pose.position.x = center_x;
    box.pose.position.y = center_y;
    box.pose.position.z = center_z;
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

    size_y = (size_y <= 0.1f) ? 0.1f : size_y;
    size_x = (size_x <= 0.1f) ? 0.1f : size_x;

    float half_x = (0.5) * size_x;
    float half_y = (0.5) * size_y;
    float half_z = (0.5) * size_z;

    p1.x = half_x;
    p1.y = half_y;
    p1.z = half_z;
    p2.x = half_x;
    p2.y = -half_y;
    p2.z = half_z;
    p3.x = -half_x;
    p3.y = -half_y;
    p3.z = half_z;
    p4.x = -half_x;
    p4.y = half_y;
    p4.z = half_z;
    p5 = p1;
    p5.z = -half_z;
    p6 = p2;
    p6.z = -half_z;
    p7 = p3;
    p7.z = -half_z;
    p8 = p4;
    p8.z = -half_z;

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

void Callback3(const nav_msgs::Odometry::ConstPtr& msg){
    currentOdom = *msg;
    Eigen::Vector4d m(currentOdom.pose.pose.position.x, currentOdom.pose.pose.position.y, currentOdom.twist.twist.linear.x, currentOdom.twist.twist.linear.y);
    Self.measurementUpdate(m ,currentOdom.header.stamp);
    currentOdom.pose.pose.position.x = Self.getPosX();
    currentOdom.pose.pose.position.y = Self.getPosY();
    currentOdom.twist.twist.linear.x = Self.getVelX();
    currentOdom.twist.twist.linear.y = Self.getVelY();
    tf::Transform transform;
    tf::Quaternion orientation;
    orientation.setRPY(0, 0, -1.0166);
    transform.setRotation(orientation);
    transform.setOrigin(tf::Vector3(msg -> pose.pose.position.x, msg -> pose.pose.position.y, msg -> pose.pose.position.z));
    tf_pub -> sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "vehicle"));
}

int main(int argc, char** argv){

    ros::init(argc, argv, "reproducer");
	ros::NodeHandle nh;
    // ros::Subscriber visobj = nh.subscribe("/ibeo_objects_vis", 10, Callback1);
    ros::Subscriber ibeoobj = nh.subscribe("/ibeo_objects", 10, Callback2);
    ros::Subscriber vehiclepose = nh.subscribe("/vehicle_odom", 10, Callback3);
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/object_car", 10);
    // ros::Publisher pub2 = nh.advertise<visualization_msgs::MarkerArray>("/ibeo_objects_vis_r", 10);
    ros::Publisher pub3 = nh.advertise<ibeo_8l_msgs::ObjectListLuxRos>("/track_objects", 10);
    carpub = &pub;
    // objpub = &pub2;
    trackpub = &pub3;
    tf_pub = new tf::TransformBroadcaster();
    ros::spin();
    return 0;
}