#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/common.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

using namespace std;

ros::Publisher cloudpub;
ros::Publisher planepub;
double searchradius;
int minsize;
int maxsize;

visualization_msgs::Marker createWireframeMarker(const float& x1, const float& y1,
												const float& x2, const float& y2, const float& angle){

        // minpt.x * cos(-angle) - minpt.y * sin(-angle),
        // minpt.x * sin(-angle) + minpt.y * cos(-angle),
        // maxpt.x * cos(-angle) - maxpt.y * sin(-angle),
        // maxpt.x * sin(-angle) + maxpt.y * cos(-angle)

	visualization_msgs::Marker box;
	geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    float cos_angle = cos(angle), sin_angle = sin(angle);

    int size_z = 0.5;

	p1.x = x1 * cos_angle - y1 * sin_angle;
	p1.y = x1 * sin_angle + y1 * cos_angle;
	p1.z = size_z;

	p2.x = x1 * cos_angle - y2 * sin_angle;
	p2.y = x1 * sin_angle + y2 * cos_angle;
	p2.z = size_z;

	p3.x = x2 * cos_angle - y2 * sin_angle;
	p3.y = x2 * sin_angle + y2 * cos_angle;
	p3.z = size_z;
    
	p4.x = x2 * cos_angle - y1 * sin_angle;
	p4.y = x2 * sin_angle + y1 * cos_angle;
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


void Callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (msg);//创建点云索引向量，用于存储实际的点云信息

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (searchradius); //设置近邻搜索的搜索半径
    ec.setMinClusterSize (minsize);//设置一个聚类需要的最少点数目
    ec.setMaxClusterSize (maxsize); //设置一个聚类需要的最大点数目
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (msg);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster;
    cloud_cluster.is_dense = true;
    cloud_cluster.header.frame_id = "vehicle";

    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_2nd;

    uint8_t color[3] = {}, c1 = 0, c2 = 0;

    visualization_msgs::MarkerArray marker_array;
    uint i = 0, j = 0;
            
    inliers = (pcl::PointIndices::Ptr)new pcl::PointIndices;
    cloud_object_2nd = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointXYZ p;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        c2 += 64;
        if(c2 >= 256){
            c2 = 0;
            c1 ++;
            c1 %= 3;
        }
        color[0] = color[1] = color[2] = 0;
        color[c1] = c2;
        cloud_object = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

        cloud_object -> width = it -> indices.size();
        cloud_object -> height = 1;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            p = msg->points[*pit];
            p.z = 0.5;
            cloud_object -> points.push_back(p);
        }

        seg.setInputCloud (cloud_object);
        seg.segment (*inliers, *coef);

        cout << coef -> values[0] << " " << coef -> values[1] << " " << coef -> values[2] << " "  
             << coef -> values[3] << " " << coef -> values[4] << " " << coef -> values[5] << " "  << endl;
        
        // visualization_msgs::Marker marker;
        // marker.id = ++i;
        // marker.header.frame_id = "vehicle";
        // marker.type = visualization_msgs::Marker::LINE_STRIP;
        // marker.header.stamp = ros::Time::now();
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.lifetime = ros::Duration(0.5);
        // marker.scale.x = 0.2;
        // marker.color.b = 1.0;
        // marker.color.a = 1.0;

        // geometry_msgs::Point p;
        // p.x = coef -> values[0];
        // p.y = coef -> values[1];
        // p.z = coef -> values[2];
        // marker.points.push_back(p);
        // p.x = coef -> values[0] + coef -> values[3] * 2;
        // p.y = coef -> values[1] + coef -> values[4] * 2;
        // p.z = coef -> values[2] + coef -> values[5] * 2;
        // marker.points.push_back(p);
        // marker_array.markers.push_back(marker);


        cout << coef -> values[0] << " " << coef -> values[1] << " " << coef -> values[2] << " "  
             << coef -> values[3] << " " << coef -> values[4] << " " << coef -> values[5] << " "  << endl;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float angle = atan2(coef -> values[3], coef -> values[4]);
        transform(0,0) = cos(angle);
        transform(0,1) = -sin(angle);
        transform(1,0) = sin(angle);
        transform(1,1) = cos(angle);
        pcl::transformPointCloud (*cloud_object, *cloud_object_2nd, transform);
        pcl::PointXYZ minpt, maxpt;
        // pcl::getMinMax3D(*cloud_object_2nd, minpt, maxpt);
        pcl::getMinMax3D(*cloud_object_2nd, minpt, maxpt);
        // visualization_msgs::Marker box = createWireframeMarker(minpt.x, minpt.y, maxpt.x, maxpt.y);
        visualization_msgs::Marker box = createWireframeMarker(
            minpt.x, minpt.y, maxpt.x, maxpt.y, -angle
        );

        box.id = ++i;
        box.header.frame_id = "vehicle";
        box.type = visualization_msgs::Marker::LINE_LIST;
        box.header.stamp = ros::Time::now();
        box.action = visualization_msgs::Marker::ADD;
        box.lifetime = ros::Duration(0.5);
        box.scale.x = 0.1;
        box.color.b = 1.0;
        box.color.a = 1.0;

        marker_array.markers.push_back(box);

        // for (j = 0; j < cloud_object_2nd -> points.size(); ++j)
        // {
        //     pcl::PointXYZ p = cloud_object_2nd->points[j];
        //     pcl::PointXYZRGB pr;
        //     pr.x = p.x;
        //     pr.y = p.y;
        //     pr.z = p.z;
        //     pr.r = color[0];
        //     pr.g = color[1];
        //     pr.b = color[2];
        //     //cout << "x=" << pr.x << " ,y=" << pr.y << " ,z=" << pr.z << endl;
        //     cloud_cluster.points.push_back(pr);
        // }


        // extract.setInputCloud(cloud_object);
        // extract.setIndices(inliers);
        // extract.setNegative(true);
        // extract.filter(*cloud_object_2nd);

        // seg.setInputCloud (cloud_object_2nd);
        // seg.segment (*inliers, *coef);

        // visualization_msgs::Marker marker_2nd;
        // marker_2nd.id = ++i;
        // marker_2nd.header.frame_id = "vehicle";
        // marker_2nd.type = visualization_msgs::Marker::LINE_STRIP;
        // marker_2nd.header.stamp = ros::Time::now();
        // marker_2nd.action = visualization_msgs::Marker::ADD;
        // marker_2nd.lifetime = ros::Duration(0.5);
        // marker_2nd.scale.x = 0.2;
        // marker_2nd.color.r = 1.0;
        // marker_2nd.color.a = 1.0;
        // p.x = coef -> values[0];
        // p.y = coef -> values[1];
        // p.z = coef -> values[2];
        // marker_2nd.points.push_back(p);
        // p.x = coef -> values[0] + coef -> values[3] * 2;
        // p.y = coef -> values[1] + coef -> values[4] * 2;
        // p.z = coef -> values[2] + coef -> values[5] * 2;
        // marker_2nd.points.push_back(p);

        // marker_array.markers.push_back(marker_2nd);


        //extract.setIndices((pcl::PointIndices::ConstPtr)it);

        // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        // {
        //     pcl::PointXYZ p = msg->points[*pit];
        //     pcl::PointXYZRGB pr;
        //     pr.x = p.x;
        //     pr.y = p.y;
        //     pr.z = p.z;
        //     pr.r = color[0];
        //     pr.g = color[1];
        //     pr.b = color[2];
        //     //cout << "x=" << pr.x << " ,y=" << pr.y << " ,z=" << pr.z << endl;
        //     cloud_cluster.points.push_back(pr);
        // }
    }
    cloud_cluster.width = cloud_cluster.points.size();
    cloud_cluster.height = 1;


    ROS_INFO_STREAM("Segmentation done. Params: R=" << searchradius << " Min#=" << minsize << " Max#=" << maxsize);
    cloudpub.publish(cloud_cluster);
    planepub.publish(marker_array);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "cloud_extraction");

	ros::NodeHandle nh;
    ros::Subscriber scancloud = nh.subscribe("/ibeo_scan_up", 10, Callback);
    cloudpub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/ibeo_scan_seg", 10);
    planepub = nh.advertise<visualization_msgs::MarkerArray>("/pcl_extract", 10);

    nh.param<double>("/cloud_extraction/search_radius", searchradius, 1.0);
    nh.param<int>("/cloud_extraction/min_size", minsize, 30);
    nh.param<int>("/cloud_extraction/max_size", maxsize, 1000);
    
    ros::Rate r(20);
    while(ros::ok()){
        nh.getParam("/cloud_extraction/search_radius", searchradius);
        nh.getParam("/cloud_extraction/min_size", minsize);
        nh.getParam("/cloud_extraction/max_size", maxsize);
        ros::spinOnce();
        r.sleep();
    }
    
}