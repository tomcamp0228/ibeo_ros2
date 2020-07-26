#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <object_tracker/cloud_extraction.h>
#include <object_tracker/object_box_kalman_filter.h>
#include <ibeo_8l_msgs/msg/object_list_lux_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
// #include <visual_detect/VisualResult.h>

using namespace std;

///////////////////////////////////////////////////////////////////////////////

#define ASSOCIATION_DISTANCE_THRESHOLD (2.5)//original2.0
#define UPDATE_DISTANCE_THRESHOLD (2.0)//original1.5
#define IBEO_VEHICLE_POSITION (0)//original 1.6
string USING_FRAME = "vehicle";
class Object_Tracker :public rclcpp::Node
{
    private:
    rclcpp::Publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>::SharedPtr ibeo_msg_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scancloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub;

    std::vector<object_tracker::PCLExtractBox> observations;
    nav_msgs::msg::Odometry odom;
    double vehicle_yaw=0;
    rclcpp::Time current_time;
    std::vector<object_tracker::ObjectBoxKalmanFilter> objects;
    bool id_occupancy[1000] = {};
    uint id_front = 0;
    uint related_observation_count;
    uint related_observation[40];
    std::string scan_cloud_topic,odom_sub_topic;

    bool is_odom_received = false;
    bool is_using_world_frame = false;
    // bool is_visual_received = false;
    public:
    Object_Tracker():
    Node("object_tracker")
    {
        //由于namespace的特性，Subscribe的消息名前需加入/namespace/topic,引入parameter加以定义.
        this->declare_parameter<std::string>("scan_cloud_topic","/ibeo_scan_up");
        this->declare_parameter<std::string>("odom_sub_topic","/vehicle_odom");
        this->get_parameter_or<std::string>("scan_cloud_topic",scan_cloud_topic,"/ibeo_scan_up");
        this->get_parameter_or<std::string>("odom_sub_topic",odom_sub_topic,"/vehicle_odom");
        ibeo_msg_pub=this->create_publisher<ibeo_8l_msgs::msg::ObjectListLuxRos>("ibeo_objects_retrack",10);
        box_pub=this->create_publisher<visualization_msgs::msg::MarkerArray>("extraction",10);
        scancloud=this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_cloud_topic,10,std::bind(&Object_Tracker::lidarProcess,this,std::placeholders::_1));
        odomsub=this->create_subscription<nav_msgs::msg::Odometry>(odom_sub_topic,10,std::bind(&Object_Tracker::odomReceive,this,std::placeholders::_1));
    }

    void odomReceive(const nav_msgs::msg::Odometry::SharedPtr msg){

        odom = *msg;
        vehicle_yaw = atan2(
            2 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
            1 - 2 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z)
        );
        is_odom_received = true;
    }

    inline double calPointDistance(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    inline double getUniformYaw(double yaw){
        while(yaw < - M_PI) yaw += 2 * M_PI;
        while(yaw >= M_PI) yaw -= 2 * M_PI;
        return yaw;
    }

    inline void calBoxConversion(double &x, double &y, double size_x, double size_y, double yaw, uint from, uint to){

        while(from != to){
            from = (from + 1) % 4;
            switch(from){
                case 0:
                    x = x + size_x * cos(yaw + M_PI);
                    y = y + size_x * sin(yaw + M_PI);
                    break;
                case 1:
                    x = x + size_y * cos(yaw - M_PI_2);
                    y = y + size_y * sin(yaw - M_PI_2);
                    break;
                case 2:
                    x = x + size_x * cos(yaw);
                    y = y + size_x * sin(yaw);
                    break;
                case 3:
                    x = x + size_y * cos(yaw + M_PI_2);
                    y = y + size_y * sin(yaw + M_PI_2);
                    break;
            }
        }
    }

    void ibeoObjectPublisher(){

        ibeo_8l_msgs::msg::ObjectListLuxRos ibeo_msg;
        ibeo_msg.header.stamp = current_time;
        ibeo_msg.scan_start_timestamp = current_time;
        ibeo_msg.header.frame_id = USING_FRAME;
        uint n = objects.size();

        for(uint i = 0; i < n; i ++){
            
            if(objects[i].getTrackDuration() > rclcpp::Duration(1)){
                ibeo_8l_msgs::msg::ObjectLuxRos obj;
                obj.id = objects[i].id;
                obj.classification = objects[i].classification;
                obj.object_box_orientation = objects[i].getYaw();
                obj.object_box_size.x = objects[i].getSizeX();
                obj.object_box_size.y = objects[i].getSizeY();
                obj.object_box_center.x = objects[i].center_x;
                obj.object_box_center.y = objects[i].center_y;
                obj.absolute_velocity.x = objects[i].getRefVelocityX();
                obj.absolute_velocity.y = objects[i].getRefVelocityY();
                ibeo_msg.object_list.push_back(obj);
            }
        }
        ibeo_msg.number_of_objects = ibeo_msg.object_list.size();
        ibeo_msg_pub->publish(ibeo_msg);
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

    void boxPublisher(){
        
        visualization_msgs::msg::MarkerArray box_marker_list;
        uint i;

        for(std::vector<object_tracker::PCLExtractBox>::const_iterator it = observations.begin (); it != observations.end (); ++it){

            visualization_msgs::msg::Marker box = createWireframeMarker(
            it -> x[0], it -> y[0],
            it -> x[1], it -> y[1],
            it -> x[2], it -> y[2],
            it -> x[3], it -> y[3]
            );
            box.id = ++i;
            box.header.frame_id = USING_FRAME;
            box.type = visualization_msgs::msg::Marker::LINE_LIST;
            box.header.stamp = this->get_clock()->now();
            box.action = visualization_msgs::msg::Marker::ADD;
            box.lifetime = rclcpp::Duration(0.5);//TODO:check whether it's available
            box.scale.x = 0.1;
            box.color.b = 1.0;
            box.color.a = 1.0;
            box_marker_list.markers.push_back(box);

            visualization_msgs::msg::Marker marker_label;
            marker_label.id  = ++i;
            marker_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker_label.action = visualization_msgs::msg::Marker::MODIFY;
            marker_label.pose.position.x = it ->center_x;
            marker_label.pose.position.y = it ->center_y;
            marker_label.pose.position.z = 0.5;
            marker_label.text = to_string((it->x[0])) + ", " + to_string((it->y[0])) +'\n' + to_string((it->x[1])) + ", " + to_string((it->y[1]));
            marker_label.scale.x = 0.1;
            marker_label.scale.y = 0.1;
            marker_label.scale.z = 0.5;
            marker_label.lifetime = rclcpp::Duration(0.5);//TODO:check whether it's available
            marker_label.color.r = marker_label.color.g = marker_label.color.b = 1;
            marker_label.color.a = 0.5;
            marker_label.header.stamp = this->get_clock()->now();
            marker_label.header.frame_id = USING_FRAME;
            box_marker_list.markers.push_back(marker_label);
        }
        box_pub->publish(box_marker_list);
    }

    void transferToWorldCoordinate(){
        
        uint n = observations.size();
        double cos_yaw = cos(vehicle_yaw), sin_yaw = sin(vehicle_yaw);
        double tmp_x , tmp_y;
        uint i, j;
        for(i = 0; i < n; i ++){
            for(j = 0; j < 4; j ++){
                tmp_x += IBEO_VEHICLE_POSITION;
                tmp_x = observations[i].x[j] * cos_yaw - observations[i].y[j] * sin_yaw;
                tmp_y = observations[i].x[j] * sin_yaw + observations[i].y[j] * cos_yaw;
                tmp_x += odom.pose.pose.position.x;
                tmp_y += odom.pose.pose.position.y;
                observations[i].x[j] = tmp_x;
                observations[i].y[j] = tmp_y;
            }
            observations[i].yaw = observations[i].yaw + vehicle_yaw;
            observations[i].reCalBox();
        }
    }

    void lidarProcess(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg,*pcl_cloud);
        observations = object_tracker::extractBox(pcl_cloud);
        RCLCPP_INFO(this->get_logger(),"Extracted!");

        uint i, j, observation_n = observations.size(), object_n = objects.size();
        
        //if receive odom msg, change to map frame and delete all objects
        if(is_odom_received && !is_using_world_frame){
            is_using_world_frame = true;
            std::vector<object_tracker::ObjectBoxKalmanFilter>::iterator it = objects.begin();
            while(it != objects.end())
                it = objects.erase(it);
            USING_FRAME = "map";
            return;
        }

        if(is_using_world_frame){
            transferToWorldCoordinate();
            current_time = odom.header.stamp;
        }
        else
            current_time = this->get_clock()->now();
        //using contour points
        for(i = 0; i < object_n; i ++){
            
            objects[i].prediction(current_time);    //do the prediction of kalman filter
            objects[i].calContourPoints();

            //rough classification
            bool is_visual_classified = false;
            double tmp_visual_angle = atan2(objects[i].center_y - (odom.pose.pose.position.y + 1.2 * sin(vehicle_yaw)),
                                            objects[i].center_x - (odom.pose.pose.position.x + 1.2 * cos(vehicle_yaw)));
            tmp_visual_angle -= vehicle_yaw;
            tmp_visual_angle = getUniformYaw(tmp_visual_angle);
            // if(is_visual_received){
            //     // if(objects[i].id == 2){
            //         cout << "id:" << objects[i].id << endl;
            //         cout <<"lidar angle" << tmp_visual_angle << endl;
            //         cout <<"visual angles ";
            //         for(j = 0; j < vehicle_visual_detected.vehicle_detected.size(); j ++){
            //             cout << vehicle_visual_detected.vehicle_detected[j] << " ";
            //         }
            //         cout << endl;
            //     // }

            // }
            

            // for(j = 0; j < vehicle_visual_detected.vehicle_detected.size() && is_visual_received; j ++){
            //     if(fabs(tmp_visual_angle - vehicle_visual_detected.vehicle_detected[j]) < 0.122){
            //         is_visual_classified = true;
            //         break;
            //     }
            // }
            // cout <<"size:x" << objects[i].getSizeX() << "size:y" << objects[i].getSizeY() << endl;
            if(objects[i].getSizeX() < 4.0 && objects[i].getSizeY() < 4.0){
                // if(objects[i].getSizeX() > objects[i].getSizeY()){
                //     if(objects[i].getSizeY() < 2.0 && objects[i].getSizeY() > 1.0 && is_visual_classified){
                //         objects[i].classification = object_tracker::ObjectBoxKalmanFilter::CAR;
                //     }
                // }
                // else{
                //     if(objects[i].getSizeX() < 2.0 && objects[i].getSizeX() > 1.0 && is_visual_classified){
                //         objects[i].classification = object_tracker::ObjectBoxKalmanFilter::CAR;
                //     }
                // }
                if((objects[i].getSizeY() < 2.0 && objects[i].getSizeY() > 1.0 && is_visual_classified) ||
                (objects[i].getSizeX() < 2.0 && objects[i].getSizeX() > 1.0 && is_visual_classified)){
                    objects[i].classification = object_tracker::ObjectBoxKalmanFilter::CAR;
                }
            }
            else{
                objects[i].classification = object_tracker::ObjectBoxKalmanFilter::UNKNOWN_BIG;
            }

            //rough yaw adjustment
            // if(sqrt(pow(objects[i].getRefVelocityY(), 2) + pow(objects[i].getRefVelocityX(), 2)) > 1.5 &&
            //    objects[i].classification == object_tracker::ObjectBoxKalmanFilter::CAR){

            //     double tmp_yaw = objects[i].getYaw(), velocity_yaw = atan2(objects[i].getRefVelocityY(), objects[i].getRefVelocityX());
            //     for(j = 0; j < 4; j ++){
            //         if(fabs(getUniformYaw(tmp_yaw - velocity_yaw)) < 0.5){
            //             objects[i].rotatePointId(j);
            //             break;
            //         }
            //         tmp_yaw += M_PI_2;
            //     }
            // }


            //rough association with center distance
            related_observation_count = 0;
            for(j = 0; j < observation_n; j ++){
                if(calPointDistance(observations[j].center_x, observations[j].center_y, objects[i].center_x, objects[i].center_y)
                    < ASSOCIATION_DISTANCE_THRESHOLD){
                    related_observation[related_observation_count ++] = j;
                    observations[j].new_observation = false;
                }
            }
        
            if(related_observation_count == 0){
                if(objects[i].is_lost == false){
                    objects[i].is_lost = true;
                    objects[i].lost_time = current_time;
                }
                continue;
            }
            else{
                objects[i].is_lost = false;
            }

            //update the yaw angle
            double new_yaw = 99999.0;
            for (j = 0; j < related_observation_count; j ++){
                while(observations[related_observation[j]].yaw > objects[i].getYaw()) observations[related_observation[j]].yaw -= M_PI * 2;
                while(objects[i].getYaw() - observations[related_observation[j]].yaw > M_PI_4) observations[related_observation[j]].rotatePointIndex(1);

                if(fabs(observations[related_observation[j]].yaw - objects[i].getYaw()) < fabs(new_yaw - objects[i].getYaw())){
                    new_yaw = observations[related_observation[j]].yaw;
                }
            }
            // if(fabs(new_yaw - objects[i].getYaw()) < 0.35) 
                objects[i].yawMeasurementUpdate(new_yaw, current_time);
            
            //update the position with contour points
            double tmp_dist, tmp_x, tmp_y, update_x = 0, update_y = 0;
            bool contour_fit[20][4] = {};
            uint update_count = 0;

            //update the ref_point_id
            double ref_angle = atan2(objects[i].center_y - odom.pose.pose.position.y, objects[i].center_x - odom.pose.pose.position.x);
            ref_angle -= objects[i].getYaw();
            ref_angle = getUniformYaw(ref_angle);

            uint ref_point_id = 4;
            if(ref_angle >= 0 && ref_angle < M_PI_2){
                ref_point_id = 1;
            }
            else if(ref_angle >= M_PI_2 && ref_angle < M_PI){
                ref_point_id = 2;
            }
            else if(ref_angle < 0 && ref_angle >= - M_PI_2){
                ref_point_id = 0;
            }
            else if(ref_angle < - M_PI_2 && ref_angle >= - M_PI){
                ref_point_id = 3;
            }

            double ref_point_lidar_angle = atan2(objects[i].y[ref_point_id] - odom.pose.pose.position.y,
                                                objects[i].x[ref_point_id] - odom.pose.pose.position.x);
            ref_point_lidar_angle = getUniformYaw(ref_point_lidar_angle - vehicle_yaw);
            if(ref_point_lidar_angle > M_PI_4 || ref_point_lidar_angle < - M_PI_4){
                double ref2_point_lidar_angle = atan2(objects[i].y[(ref_point_id + 3) % 4] - odom.pose.pose.position.y,
                                                    objects[i].x[(ref_point_id + 3) % 4] - odom.pose.pose.position.x);
                ref2_point_lidar_angle = getUniformYaw(ref2_point_lidar_angle - vehicle_yaw);
                if(fabs(ref2_point_lidar_angle) < M_PI_4) ref_point_id = (ref_point_id + 3) % 4;
                else{
                    ref2_point_lidar_angle = atan2(objects[i].y[(ref_point_id + 1) % 4] - odom.pose.pose.position.y,
                                                objects[i].x[(ref_point_id + 1) % 4] - odom.pose.pose.position.x);
                    ref2_point_lidar_angle = getUniformYaw(ref2_point_lidar_angle - vehicle_yaw);
                    if(fabs(ref2_point_lidar_angle) < M_PI_4) ref_point_id = (ref_point_id + 1) % 4;
                }
            }

            if(ref_point_id != objects[i].ref_point_id)
                objects[i].changeRefPoint(ref_point_id);

            for (j = 0; j < related_observation_count; j ++){

                    tmp_x = observations[related_observation[j]].x[ref_point_id];
                    tmp_y = observations[related_observation[j]].y[ref_point_id];
                    // calBoxConversion(tmp_x, tmp_y, objects[i].getSizeX(), objects[i].getSizeY(), objects[i].getYaw(), k, 0);
                    tmp_dist = calPointDistance(tmp_x, tmp_y, objects[i].getRefPositionX(), objects[i].getRefPositionY());
                    if(tmp_dist < UPDATE_DISTANCE_THRESHOLD){
                        contour_fit[j][ref_point_id] = true;
                        update_x += tmp_x;
                        update_y += tmp_y;
                        update_count ++;
                    }
            }
            bool size_updated = false;
            if(update_count > 0){
                update_x /= update_count;
                update_y /= update_count;
                
                objects[i].positionMeasurementUpdate(update_x, update_y, current_time);

                for (j = 0; j < related_observation_count; j ++){
                    if((contour_fit[j][0] && contour_fit[j][2]) || (contour_fit[j][1] && contour_fit[j][3])){
                        if((contour_fit[j][0] && contour_fit[j][2])){
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 0);
                        }
                        else{
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 1);
                        }
                        size_updated = true;
                    }
                    else if((contour_fit[j][0] && contour_fit[j][1]) || (contour_fit[j][2] && contour_fit[j][3])){
                        if((contour_fit[j][0] && contour_fit[j][1])){
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 0);
                        }
                        else{
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 2);
                        }
                        size_updated = true;
                    }
                    else if((contour_fit[j][0] && contour_fit[j][3]) || (contour_fit[j][1] && contour_fit[j][2])){
                        if((contour_fit[j][0] && contour_fit[j][3])){
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 0);
                        }
                        else{
                            objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, 1);
                        }
                        size_updated = true;
                    }
                }
                if(!size_updated){
                    for (j = 0; j < related_observation_count; j ++){
                        uint fit_id = 5;
                        for(uint k = 0; k < 4; k ++){
                            if(contour_fit[j][k]){
                                fit_id = k;
                                break;
                            }
                        }
                        if(fit_id != 5){
                            if(observations[related_observation[j]].size_x >= objects[i].getSizeX() ||
                                observations[related_observation[j]].size_y >= objects[i].getSizeY()){
                                    objects[i].updateSize(observations[related_observation[j]].size_x, observations[related_observation[j]].size_y, fit_id);
                                }
                        }
                    }
                }
            }
            else{
                if(objects[i].is_lost == false){
                    objects[i].is_lost = true;
                    objects[i].lost_time = current_time;
                }
            }
        }

        //reset the visual receive situation
        // if(is_visual_received) is_visual_received = false;

        //using center
        // for(i = 0; i < object_n; i ++){
            
        //     objects[i].prediction(current_time);    //do the prediction of kalman filter
        //     objects[i].calContourPoints();

        //     //rough association with center distance
        //     related_observation_count = 0;
        //     for(j = 0; j < observation_n; j ++){
        //         if(calPointDistance(observations[j].center_x, observations[j].center_y, objects[i].center_x, objects[i].center_y)
        //             < ASSOCIATION_DISTANCE_THRESHOLD){
        //             related_observation[related_observation_count ++] = j;
        //             observations[j].new_observation = false;
        //         }
                
        //     }
        
        //     if(related_observation_count == 0){
        //         if(objects[i].is_lost == false){
        //             objects[i].is_lost = true;
        //             objects[i].lost_time = current_time;
        //         }
        //         continue;
        //     }
        //     else{
        //         objects[i].is_lost = false;
        //     }

        //     //update the yaw angle
        //     double new_yaw = 99999.0;
        //     for (j = 0; j < related_observation_count; j ++){
        //         while(observations[related_observation[j]].yaw > objects[i].getYaw()) observations[related_observation[j]].yaw -= M_PI * 2;
        //         while(objects[i].getYaw() - observations[related_observation[j]].yaw > M_PI_4) observations[related_observation[j]].rotatePointIndex(1);

        //         if(fabs(observations[related_observation[j]].yaw - objects[i].getYaw()) < fabs(new_yaw - objects[i].getYaw())){
        //             new_yaw = observations[related_observation[j]].yaw;
        //         }
        //     }
        //     // if(fabs(new_yaw - objects[i].getYaw()) < 0.35) 
        //         objects[i].yawMeasurementUpdate(new_yaw, current_time);
            
        //     //update the position
        //     uint new_pos_id;
        //     double new_pos_dist = 99999.0, tmp_dist;
        //     for (j = 0; j < related_observation_count; j ++){

        //         tmp_dist = calPointDistance(observations[related_observation[j]].center_x, observations[related_observation[j]].center_y,
        //                             objects[i].getRefPositionX(), objects[i].getRefPositionY());
        //         if(tmp_dist < new_pos_dist){
        //             new_pos_dist = tmp_dist;
        //             new_pos_id = j;
        //         }
        //     }
        //     if(new_pos_dist < 1.2){
        //         objects[i].positionMeasurementUpdate(observations[related_observation[new_pos_id]].center_x,
        //                                              observations[related_observation[new_pos_id]].center_y, current_time);
        //         objects[i].updateSize(observations[related_observation[new_pos_id]].size_x, observations[related_observation[new_pos_id]].size_y, 4);
        //     }
        // }

        //del old objects
        std::vector<object_tracker::ObjectBoxKalmanFilter>::iterator it = objects.begin();
        while(it != objects.end()){
            if(it -> is_lost/*&& ((current_time - it -> lost_time) > ros::Duration(0.5))*/){
                it = objects.erase(it);
            }
            else ++it;
        }

        //del repeated objects
        it = objects.begin();
        while(it != objects.end()){
            std::vector<object_tracker::ObjectBoxKalmanFilter>::iterator it_j = it + 1;
            bool is_del = false;
            while(it_j != objects.end()){
                if(calPointDistance(it -> getRefPositionX(), it -> getRefPositionY(), it_j -> getRefPositionX(), it_j -> getRefPositionY()) < 0.10){
                    it = objects.erase(it);
                    is_del = true;
                    break;
                }
                it_j ++;
            }
            if(!is_del) it ++;
        }

        //create new objects
        for(i = 0; i < observation_n; i ++){
            if(observations[i].new_observation){
                if(id_front >= 1000) id_front = 0;
                cout << "add: pos:" << observations[i].x[0] << ", " << observations[i].y[0] << endl;
                cout << "add: size: "<< observations[i].size_x << ", " << observations[i].size_y << endl;

                //using contour points
                double ref_angle = atan2(observations[i].center_y, observations[i].center_x);
                ref_angle -= observations[i].yaw;
                while(ref_angle < - M_PI) ref_angle += 2 * M_PI;
                while(ref_angle >= M_PI) ref_angle -= 2 * M_PI;
                uint ref_point_id = 4;
                if(ref_angle >= 0 && ref_angle < M_PI_2){
                    ref_point_id = 1;
                }
                else if(ref_angle >= M_PI_2 && ref_angle < M_PI){
                    ref_point_id = 2;
                }
                else if(ref_angle < 0 && ref_angle >= - M_PI_2){
                    ref_point_id = 0;
                }
                else if(ref_angle < - M_PI_2 && ref_angle >= - M_PI){
                    ref_point_id = 3;
                }
                object_tracker::ObjectBoxKalmanFilter newobj(id_front ++, observations[i].size_x, observations[i].size_y,
                                                                        observations[i].x[ref_point_id], observations[i].y[ref_point_id],
                                                                        observations[i].yaw, current_time, ref_point_id);
                //using center
                // object_tracker::ObjectBoxKalmanFilter newobj(id_front ++, observations[i].size_x, observations[i].size_y,
                //                                             observations[i].center_x, observations[i].center_y,
                //                                             observations[i].yaw, current_time, 4);
                objects.push_back(newobj);
                objects.back().calContourPoints();
            }
        }

        ibeoObjectPublisher();
        boxPublisher();
    }


};





int main(int argc, char** argv){

    rclcpp::init(argc,argv);
    auto object_tracker=std::make_shared<Object_Tracker>();

    rclcpp::spin(object_tracker);
    return 0;
    
}