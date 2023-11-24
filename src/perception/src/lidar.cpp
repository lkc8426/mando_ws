#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>




class Lidar{
    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub;
    ros::Subscriber lidar_sub;
    ros::Publisher output_pub;
    pcl::PointCloud<pcl::PointXYZ> lidar_point;

public:
    Lidar();

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(pcl::PointCloud<pcl::PointXYZ> cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inlierPoint_neg);
    
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg);
    void object_detection();
};


Lidar::Lidar(){
    lidar_sub = nh.subscribe("/carla/ego_vehicle/lidar", 1 , &Lidar::pcl_callback, this);
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::voxel(pcl::PointCloud<pcl::PointXYZ> cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud (cloud.makeShared());
    voxel_filter.setLeafSize (0.4, 0.4, 0.4);
    voxel_filter.filter(*filter_cloud);

    return filter_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());
  
  // SACSegmentation 을 위해서 seg 를 만들고 방법과 모델과 기준을 정함
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 적용 모델
    seg.setMethodType(pcl::SAC_RANSAC); // 적용 방법
    seg.setMaxIterations(1000); //최대 실행 수

    seg.setDistanceThreshold(0.1); //inlier로 처리할 거리 정보
    seg.setInputCloud(voxel_cloud); //setinputcloud에는 포인터 넣는 거인듯?? [setInputCloud (const PointCloudConstPtr &cloud)]
    seg.segment(*inliers,*coefficients);

    // input cloud에서 평면 inlier를 추출함.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(voxel_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);           //바닥제거하기 위해 inlier를 없앤다
    extract.filter(*inlierPoints_neg);

    return inlierPoints_neg;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Lidar::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inlierPoints_neg){

  int size = inlierPoints_neg ->size();
  std::vector<double> z(size);
  for(int i = 0;i < size; i++){
    z[i] = inlierPoints_neg ->points[i].z;
    inlierPoints_neg ->points[i].z = 0.0;
  }
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (inlierPoints_neg); //kdtree 생성
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1);
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (inlierPoints_neg);
  ec.extract (cluster_indices);
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  
  for(int i = 0; i < size ; i++){
    inlierPoints_neg -> points[i].z = z[i];
  }

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;

  int cluster_num = cluster_indices.size();
  int j = 0;

  for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    xyzi_cloud -> points.reserve(it->indices.size());
    for(auto pit = it -> indices.begin(); pit != it -> indices.end(); ++pit){
      pcl::PointXYZ pt = inlierPoints_neg->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x;
      pt2.y = pt.y;
      pt2.z = pt.z;

      pt2.intensity = (float)(j+1);
      TotalCloud.push_back(pt2);
      xyzi_cloud -> points.push_back(pt2);
    }
    j++;
    clusters.push_back(xyzi_cloud);

  }

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "ego_vehicle/lidar";

  output_pub.publish(output);

  return clusters;
}

void Lidar::object_detection(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    filter_cloud = voxel(lidar_point);
    inlierPoints_neg = ground_removal(filter_cloud);
    clusters = Clustering(inlierPoints_neg);

    lidar_point.clear();
}


void Lidar::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
    // pcl::PointCloud<pcl::PointXYZ> lidar_point;
    pcl::fromROSMsg(*pc_msg,lidar_point);
}

int main(int argv, char** argc){
    ros::init(argv,argc,"lidar");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    
    Lidar lidar;

    while(ros::ok()){
        lidar.object_detection();

        loop_rate.sleep();
        ros::spinOnce();
    }
}