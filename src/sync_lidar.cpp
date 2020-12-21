#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub_sync_lidar_;

void callback(const sensor_msgs::PointCloud2::ConstPtr& velodyne_points_ptr, const sensor_msgs::PointCloud2::ConstPtr& rslidar_points_ptr)  //回调中包含多个消息
{
  //std::cout<<"has been in"<<std::endl;
  pcl::PointCloud<pcl::PointXYZI> velodyne_points;
  pcl::PointCloud<pcl::PointXYZI> rslidar_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr sync_lidar_points(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 pub_sync_lidar_points;
  pcl::fromROSMsg(*velodyne_points_ptr, velodyne_points);
  pcl::fromROSMsg(*rslidar_points_ptr, rslidar_points);
  for(uint i = 0; i <= ( velodyne_points.points.size() + rslidar_points.points.size() ); i++){
    if(i <= velodyne_points.points.size()){
      sync_lidar_points->push_back(velodyne_points.points[i]);
    }else{
      sync_lidar_points->push_back(rslidar_points.points[i - velodyne_points.points.size()]);
    }
  }
  //std::cout<<sync_lidar_points->points.size()<<std::endl;
  pcl::toROSMsg(*sync_lidar_points, pub_sync_lidar_points);

  pub_sync_lidar_points.header = rslidar_points_ptr->header;
  pub_sync_lidar_points.width = sync_lidar_points->points.size();
  pub_sync_lidar_points.height = velodyne_points_ptr->height;
  pub_sync_lidar_points.is_dense = true;

  pub_sync_lidar_.publish(pub_sync_lidar_points);

  sync_lidar_points->clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_lidar");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh, "/velodyne_points", 1);             // topic1 输入
  message_filters::Subscriber<sensor_msgs::PointCloud2> rslidar_sub(nh, "/rslidar_points", 1);   // topic2
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, rslidar_sub);
  //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(velodyne_sub, rslidar_sub, 10);       // 同步
  sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调
  pub_sync_lidar_ = nh.advertise<sensor_msgs::PointCloud2>("/sync_lidar_points", 10);

  ros::spin();

  return 0;
}
