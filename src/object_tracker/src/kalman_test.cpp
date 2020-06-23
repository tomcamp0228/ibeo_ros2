#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <object_tracker/object_box_kalman_filter.h>

using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "test_kalman");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/kalman_test", 10);

    double observe_x[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    double observe_y[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    double observe_yaw[] = {0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};

    object_tracker::ObjectBoxKalmanFilter filter(1, 3.2, 1.5, 0, 0, 0, ros::Time::now(), 2);
    

    visualization_msgs::Marker box;
    box.id = 101;
    box.header.frame_id = "map";
    box.type = visualization_msgs::Marker::LINE_STRIP;
    box.header.stamp = ros::Time::now();
    box.action = visualization_msgs::Marker::ADD;
    box.lifetime = ros::Duration(2);
    box.scale.x = 0.1;
    box.color.b = 1.0;
    box.color.a = 1.0;

    geometry_msgs::Point p;
    for(uint j = 0; j < 4; j ++){
        p.x = filter.x[j];
        p.y = filter.y[j];
        box.points.push_back(p);
    }
    pub.publish(box);

    sleep(1);

    ros::Rate r(1);
    for(uint i = 0; i < 10; i ++){
        filter.positionAndYawMeasurementUpdate(observe_x[i], observe_y[i], observe_yaw[i], ros::Time::now());
        filter.calContourPoints();
        cout << filter.getRefPositionX() << ", " << filter.getRefPositionY () << " --- " << filter.x[3] << ", " <<filter.y[3] << endl;
        visualization_msgs::Marker box;
        box.id = 101;
        box.header.frame_id = "map";
        box.type = visualization_msgs::Marker::LINE_STRIP;
        box.header.stamp = ros::Time::now();
        box.action = visualization_msgs::Marker::ADD;
        box.lifetime = ros::Duration(2);
        box.scale.x = 0.1;
        box.color.b = 1.0;
        box.color.a = 1.0;

        geometry_msgs::Point p;
        for(uint j = 0; j < 4; j ++){
            p.x = filter.x[j];
            p.y = filter.y[j];
            box.points.push_back(p);
        }
        pub.publish(box);
        r.sleep();
    }

    return 0;
}