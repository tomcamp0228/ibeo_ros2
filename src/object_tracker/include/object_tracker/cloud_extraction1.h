#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/common.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <math.h>

using namespace std;

namespace object_tracker{


double SEARCH_R = 0.7;
int CLUSTER_SIZE_MIN = 5;
int CLUSTER_SIZE_MAX = 800;

/*
    3 ___ 2
    |     |    .
    |     |   / \ yaw
    |     |    |
    |     |    |
    0-----1
*/

struct PCLExtractBox{

    double x[4];
    double y[4];//position of 1234 location points
    double yaw;
    double center_x, center_y;
    double size_x, size_y;//length & width of the vehicle
    bool new_observation;
    inline void setBox(double x0, double y0, double x1, double y1,
                            double x2, double y2, double x3, double y3, double _yaw){
        x[0] = x0; y[0] = y0; x[1] = x1; y[1] = y1;
        x[2] = x2; y[2] = y2; x[3] = x3; y[3] = y3;
        yaw = _yaw;
        center_x = (x[0] + x[2]) / 2.0;
        center_y = (y[0] + y[2]) / 2.0;
        size_x = sqrt((x[3] - x[0]) * (x[3] - x[0]) + (y[3] - y[0]) * (y[3] - y[0]));
        size_y = sqrt((x[0] - x[1]) * (x[0] - x[1]) + (y[0] - y[1]) * (y[0] - y[1]));
    }

    inline void reCalBox(){
        center_x = (x[0] + x[2]) / 2.0;
        center_y = (y[0] + y[2]) / 2.0;
        size_x = sqrt((x[3] - x[0]) * (x[3] - x[0]) + (y[3] - y[0]) * (y[3] - y[0]));
        size_y = sqrt((x[0] - x[1]) * (x[0] - x[1]) + (y[0] - y[1]) * (y[0] - y[1]));
    }

    inline void rotatePointIndex(uint n){   //to counterclockwise rotate the contour point index and yaw for n * 90 degrees 
        double x_tmp[4], y_tmp[4], swap_tmp;
        uint i; 
        for(i = 0; i < 4; i ++){
            x_tmp[i] = x[i]; y_tmp[i] = y[i];
        }
        for(i = 0; i < 4; i ++){
            x[i] = x_tmp[(i + n) % 4];
            y[i] = y_tmp[(i + n) % 4];
        }
        yaw = (yaw + (M_PI / 2) * n);
        // while(yaw > M_PI) yaw -= 2 * M_PI;
        if(n % 2 != 0){
            swap_tmp = size_x;
            size_x = size_y;
            size_y = swap_tmp;
        }
    }
};

std::vector<PCLExtractBox> extractBox(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg){
    
    //Do the Euclidean cluster extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (msg);//创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (SEARCH_R); //设置近邻搜索的搜索半径
    ec.setMinClusterSize (CLUSTER_SIZE_MIN);//设置一个聚类需要的最少点数目
    ec.setMaxClusterSize (CLUSTER_SIZE_MAX); //设置一个聚类需要的最大点数目
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (msg);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    //Prepare for the RANSAC extraction
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_2nd(new pcl::PointCloud<pcl::PointXYZI>);

    uint i = 0, j = 0;
    std::vector<PCLExtractBox> result;
    PCLExtractBox box;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        //create cloud objects
        cloud_object = (pcl::PointCloud<pcl::PointXYZI>::Ptr) new pcl::PointCloud<pcl::PointXYZI>;
        cloud_object -> width = it -> indices.size();
        cloud_object -> height = 1;
        pcl::PointXYZI p;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            p = msg->points[*pit];
            p.z = 0.5;
            cloud_object -> points.push_back(p);
        }

        seg.setInputCloud (cloud_object);
        seg.segment (*inliers, *coef);
        
        // cout << coef -> values[0] << " " << coef -> values[1] << " " << coef -> values[2] << " "  
        //     << coef -> values[3] << " " << coef -> values[4] << " " << coef -> values[5] << " "  << endl;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float angle = atan2(coef -> values[4], coef -> values[3]);
        transform(0,0) = cos(- angle);
        transform(0,1) = -sin(- angle);
        transform(1,0) = sin(- angle);
        transform(1,1) = cos(- angle);

        pcl::transformPointCloud (*cloud_object, *cloud_object_2nd, transform);
        pcl::PointXYZI minpt, maxpt;
        pcl::getMinMax3D(*cloud_object_2nd, minpt, maxpt);

        //format the box
        double cos_angle = cos(angle), sin_angle = sin(angle);
        box.setBox(
            minpt.x * cos_angle - maxpt.y * sin_angle,
            minpt.x * sin_angle + maxpt.y * cos_angle,
            minpt.x * cos_angle - minpt.y * sin_angle,
            minpt.x * sin_angle + minpt.y * cos_angle,
            maxpt.x * cos_angle - minpt.y * sin_angle,
            maxpt.x * sin_angle + minpt.y * cos_angle,
            maxpt.x * cos_angle - maxpt.y * sin_angle,
            maxpt.x * sin_angle + maxpt.y * cos_angle,
            angle
        );
        box.new_observation = true;
        result.push_back(box);
    }
    ROS_INFO_STREAM("Segmentation done.");
    return result;
}


}   //namespace object_tracker