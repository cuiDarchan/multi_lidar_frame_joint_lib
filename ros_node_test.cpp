/*
 * @Description: 测试文件
 * @Author: cuiDarchan
 * @Date: 2023-03-20
 */
#include "cloud_subscriber.hpp"


int main(int argc, char** argv){
  ros::init(argc, argv, "multi_point_cloud");
  ros::NodeHandle nh;
  std::shared_ptr<TFSubscriber> tf_subsciber = std::make_shared<TFSubscriber>(
        nh, "/fcu_controller/global_pos_int", 300);
  std::shared_ptr<CloudSubscriber> cloud_subsciber = std::make_shared<CloudSubscriber>(
        nh, "/rslidar", 30, tf_subsciber);
  ros::Publisher merge_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug_merge_points", 1000);

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ros::Rate rate(10);  // 10hz
  while (ros::ok()) {
    ros::spinOnce();
    bool success = cloud_subsciber->GetMultiFramePointCloud(5, 2, all_cloud_ptr);
    
    // TODO: debug 发布
    if(success){
        sensor_msgs::PointCloud2 debug_points;
        pcl::toROSMsg(*all_cloud_ptr, debug_points);
        debug_points.header.frame_id = "map";
        merge_points_pub.publish(debug_points);

        // std::cout << " all_cloud_ptr points size: " << all_cloud_ptr->points.size() << std::endl; // 39w
        std::cout << " all_cloud_ptr timestamp: " << std::fixed << static_cast<double>(all_cloud_ptr->header.stamp*1e-6) << std::endl;
    }

    rate.sleep();
  }

  return 0;        
}
    
