// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/segmentation/extract_clusters.h>
// #include <pcl_ros/segmentation/sac_segmentation.h>
// #include <pcl_ros/filters/extract_indices.h>
// #include <pcl_ros/transforms.h>

// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/console/time.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/conditional_euclidean_clustering.h>

// #include <pcl/common/common.h>
// #include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

using namespace std;
typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;
namespace object_tracker{


  double SEARCH_R = 0.8;// original 0.8
  int CLUSTER_SIZE_MIN = 5;// original 5
  int CLUSTER_SIZE_MAX = 800;// original 800


  /*
      3 ___ 2
      |     |    .
      |     |  / \ yaw
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


  bool enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
  {
    if (std::abs (point_a.intensity - point_b.intensity) < 0.5f)
      return (true);
    else
      return (false);
  }

  bool enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
  {
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
    if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
      return (true);
    if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
      return (true);
    return (false);
  }

  bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
  {
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
    if (squared_distance < 1.0f)
    {
      if (std::abs (point_a.intensity - point_b.intensity) < 0.25f)
        return (true);
      if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
        return (true);
    }
    else
    {
      if (std::abs (point_a.intensity - point_b.intensity) < 0.01f)
        return (true);
    }
    return (false);
  }




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

      //   //voxel grid filter
      //   pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      //   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter_out (new pcl::PointCloud<pcl::PointXYZI>);

      //   // voxel_filter.setInputCloud(msg);
      //   // voxel_filter.setLeafSize(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
      //   // voxel_filter.filter(*cloud_filter_out);


      //   pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
      //   pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
      //   pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
      //   pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
      //   pcl::copyPointCloud(*msg,*cloud_with_normals);

      //   //search_tree->setInputCloud(msg);

      //   // Set up a Normal Estimation class and merge data in cloud_with_normals
      //   pcl::NormalEstimation<PointTypeIO,PointTypeFull> ne;
      //   ne.setInputCloud(msg);
      //   ne.setSearchMethod(search_tree);
      //   ne.setRadiusSearch(10.0);
      //   ne.compute(*cloud_with_normals);

      //   // Set up a Conditional Euclidean Clustering class
      
      //  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
      //  cec.setInputCloud (cloud_with_normals);
      //  cec.setConditionFunction (&customRegionGrowing);
      //  cec.setClusterTolerance (SEARCH_R);
      //  cec.setMinClusterSize (CLUSTER_SIZE_MIN);
      //  cec.setMaxClusterSize (CLUSTER_SIZE_MAX);
      //  cec.segment (*clusters);
      //  ROS_INFO_STREAM(setw(5)<<"The size of cluster is "<<clusters->size() );
    





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
      
      for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
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

          pcl::transformPointCloud (*cloud_object, *cloud_object_2nd, transform);//rotate with point O as origin point
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
      // ROS_INFO_STREAM("Segmentation done.");
      return result;
  }


  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,Eigen::Matrix4f tranform_matrix)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_in,*pcl_cloud_transformed,tranform_matrix);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConditionAnd <pcl::PointXYZI>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZI>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT,0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT,2.2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("intensity",pcl::ComparisonOps::GT,0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("intensity",pcl::ComparisonOps::LT,FLT_MAX)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,-7.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,4.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,-30.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,50.0)));
    pcl::ConditionalRemoval<pcl::PointXYZI> condremove(range_cond);
    condremove.setInputCloud(pcl_cloud_transformed);
    condremove.setKeepOrganized(true);
    condremove.filter(*pcl_cloud_transformed_filtered);
    // RCLCPP_INFO(this->get_logger(),"00The size of point cloud is %i",pcl_cloud_transformed->points.size());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed_filtered_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_i;
    // pass_z.setInputCloud(pcl_cloud_transformed);
    // pass_z.setFilterFieldName("z");
    // pass_z.setFilterLimits(0.2,2.5);
    // pass_z.setFilterLimitsNegative(false);
    // pass_z.filter(*pcl_cloud_transformed_filtered_1);
    
    // // RCLCPP_INFO(this->get_logger(),"11The size of point cloud is %i",pcl_cloud_transformed_filtered_1->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed_filtered_2(new pcl::PointCloud<pcl::PointXYZI>);
    pass_i.setInputCloud(pcl_cloud_transformed_filtered);
    pass_i.setFilterFieldName("intensity");
    pass_i.setFilterLimits(0,FLT_MAX);
    pass_i.setFilterLimitsNegative(false);
    pass_i.filter(*pcl_cloud_transformed_filtered_2);
    // RCLCPP_INFO(this->get_logger(),"22The size of point cloud is %i",pcl_cloud_transformed_filtered_2->points.size());

    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed_filtered_3(new pcl::PointCloud<pcl::PointXYZI>);
    // pass_x.setInputCloud(pcl_cloud_transformed_filtered_2);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(-7,4);
    // pass_x.setFilterLimitsNegative(false);
    // pass_x.filter(*pcl_cloud_transformed_filtered_3);


    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed_filtered_4(new pcl::PointCloud<pcl::PointXYZI>);
    // pass_y.setInputCloud(pcl_cloud_transformed_filtered_3);
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(-30,50);
    // pass_y.setFilterLimitsNegative(false);
    // pass_y.filter(*pcl_cloud_transformed_filtered_4);

    //定义2D平面点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI p1,p2,p3,p4;
    p1.x=-3;p1.y=-3;
    p2.x=-3;p2.y=3;
    p3.x=3;p3.y=3;
    p4.x=3;p4.y=-3;
    boundingbox_ptr->push_back(p1);
    boundingbox_ptr->push_back(p2);
    boundingbox_ptr->push_back(p3);
    boundingbox_ptr->push_back(p4);
  
    pcl::ConvexHull<pcl::PointXYZI> hull;                  //创建凸包对象
    hull.setInputCloud(boundingbox_ptr);                  //设置输入点云
    hull.setDimension(2);                                 //设置凸包维度
    std::vector<pcl::Vertices> polygons;                  //设置向量，用于保存凸包定点
    pcl::PointCloud<pcl::PointXYZI>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZI>);//该点运用于描述凸包形状
    hull.reconstruct(*surface_hull, polygons);            //计算2D凸包结果
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr objects (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropHull<pcl::PointXYZI> bb_filter;               //创建crophull对象
    bb_filter.setDim(2);                                  //设置维度：该维度需要与凸包维度一致
    bb_filter.setInputCloud(pcl_cloud_transformed_filtered_2);                       //设置需要滤波的点云
    bb_filter.setHullIndices(polygons);                   //输入封闭多边形的顶点
    bb_filter.setHullCloud(surface_hull);                 //输入封闭多边形的形状
    bb_filter.filter(*objects);                           //执行CropHull滤波，存出结果在objects


    return objects;
  }


}   //namespace object_tracker