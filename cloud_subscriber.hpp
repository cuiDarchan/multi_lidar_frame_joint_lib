/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: cuiDarchan
 * @Date: 2023-03-20
 */

#pragma once

#include <vector>
#include <deque>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf_subscriber.hpp"

// 数据CloudData
class CloudData {
 public:
  using POINT = pcl::PointXYZI;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData() : cloud_ptr(new CLOUD()) {}

 public:
  double time = 0.0;
  CLOUD_PTR cloud_ptr;
};

// 点云数据订阅器
class CloudSubscriber {
 public:
  CloudSubscriber(ros::NodeHandle& nh, std::string topic_name,
                  size_t buff_size, std::shared_ptr<TFSubscriber> tf_subsciber);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData>& deque_cloud_data);
  inline int32_t getQueueSize() { return deque_size_; };
  inline void publishCloudData(const CloudData& cloud_data) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(
        new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_data.cloud_ptr, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = ros::Time(cloud_data.time);
    cloud_ptr_output->header.frame_id = "lidar_top";
    publisher_.publish(*cloud_ptr_output);
  };
  // 根据最新激光雷达时间，获取前面N帧点云拼接，且支持设置帧间隔
  bool GetMultiFramePointCloud(const int previous_frame, const int delt_frame, pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_ptr);
  

 private:
  void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::mutex buff_mutex_;  // 互斥锁

  std::deque<CloudData> new_cloud_data_;
  std::shared_ptr<TFSubscriber> tf_subsciber_;
  int32_t deque_size_ = 20;
};



CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, std::shared_ptr<TFSubscriber> tf_subsciber)
    :nh_(nh), tf_subsciber_(tf_subsciber) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::MsgCallback, this);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/merge_lidar_points", buff_size);
}


void CloudSubscriber::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    if (new_cloud_data_.size() > deque_size_ - 1) {
      new_cloud_data_.pop_front();
      new_cloud_data_.push_back(cloud_data);
    } else {
      new_cloud_data_.push_back(cloud_data);
    }
    // std::cout << " CloudSubscriber::msg_callback new_cloud_data_.size() = " << new_cloud_data_.size() << std::endl;
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();
    if (new_cloud_data_.size() > deque_size_ - 1) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    } else{
    //   std::cout << "CloudData deque size is " << new_cloud_data_.size()
    //             << " less than 5, ParseData failed!" << std::endl;
    }
    buff_mutex_.unlock();
}

bool CloudSubscriber::GetMultiFramePointCloud(const int previous_frame, const int delt_frame, pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_ptr){
  all_cloud_ptr->points.clear();
  std::deque<CloudData> cloud_data_buff;
  std::deque<TFData> tf_data_buff;
  
  ParseData(cloud_data_buff);
  tf_subsciber_->ParseData(tf_data_buff);
  
  if (cloud_data_buff.size() < getQueueSize() || tf_data_buff.size() < tf_subsciber_->getQueueSize()) return false;
  // calculate multi_point_cloud index
  std::vector<int> pc_index;
  int frame_cnt =  delt_frame * previous_frame;
  for(int i = 0; i< frame_cnt; i=i+delt_frame){
    pc_index.emplace_back(deque_size_-i-1);
    // std::cout << "pc_index: " << deque_size_-i-1 << std::endl; // 19 17 15 13 11 
  }
  // reverse(pc_index.begin(),pc_index.end());

  // 查看多帧拼接后点云，是否统一了坐标系
  bool cur_frame = true;
  CloudData all_cloud;
  Eigen::Matrix4d cur_trans = Eigen::Matrix4d::Identity();  // 当前帧旋转

  for (const auto& i : pc_index) {
    auto pc = cloud_data_buff.at(i);
    auto time = pc.time;
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

    // trans = cur-1 * next/last
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if (!cur_frame) {
      Eigen::Matrix4d nearest_trans = tf_subsciber_->FindNearestTf(tf_data_buff, time);
      trans = cur_trans.inverse() * nearest_trans;
    }
    pcl::transformPointCloud(*pc.cloud_ptr, *tmp_cloud_ptr, trans);
    *all_cloud_ptr += *tmp_cloud_ptr;

    // 第一帧记录当前帧时间、旋转矩阵，更换标记
    if (cur_frame) {
        cur_trans = tf_subsciber_->FindNearestTf(tf_data_buff, time);
        all_cloud_ptr->header.stamp = time *1e6;
        std::cout << "cur lidar time : " << time << std::endl;
        cur_frame = false;
      }
    }
    return true;
}