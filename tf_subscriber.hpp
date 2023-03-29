/*
 * @Description: 订阅imu数据
 * @Author: cuiDarchan
 * @Date: 2023-03-20
 */
#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
// #include "conversions.h"

class TFData {
  public:
    struct Translation {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    class Orientation {
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
      
      public:
        void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    double time = 0.0;
    Translation translation;
    Orientation orientation;
  
  public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<TFData>& UnsyncedData, std::deque<TFData>& SyncedData, double sync_time);
    TFData() = default;
    TFData(const Eigen::Matrix4d &data, double time){
      this->translation.x = data(0,3);
      this->translation.y = data(1,3);
      this->translation.z = data(2,3);
      Eigen::Quaterniond q(data.block<3,3>(0,0));
      this->orientation.w = q.w();
      this->orientation.x = q.x();
      this->orientation.y = q.y();
      this->orientation.z = q.z();
      this->time = time;
    };
};


class TFSubscriber {
  public:
    TFSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    TFSubscriber() = default;
    void ParseData(std::deque<TFData>& deque_imu_data);

    Eigen::Matrix4d FindNearestTf(const std::deque<TFData>& tf_data_buff, double time);
    inline int32_t getQueueSize() { return deque_size_; };
    inline int32_t getHtTFQueueSize() { return ht_deque_size_; };
    inline Eigen::Matrix4d& getMap2WorldTrans(){return map2world_trans_;};

   private:
    void MsgCallback(const nav_msgs::OdometryConstPtr& odom);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Subscriber pos_subscriber_;
    ros::Subscriber imu_subscriber_;
    std::deque<TFData> new_tf_data_;
    std::deque<TFData> imu_data_;
    std::deque<TFData> pos_data_;

    std::mutex buff_mutex_;
    std::mutex pos_buff_mutex_;
    std::mutex imu_buff_mutex_;
    int32_t deque_size_ = 250; 
    int32_t ht_deque_size_ = 20;

    Eigen::Matrix4d map2world_trans_;
    Eigen::Matrix4d world2map_trans_;
    // lidar外参
    Eigen::Matrix4d frontlidar2ins_;
    Eigen::Matrix4d downlidar2ins_;
    bool init_map2world_trans_ = false;
};


// huitian
TFSubscriber::TFSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &TFSubscriber::MsgCallback, this);
    
    // map2world
    map2world_trans_ = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q{0.38219532, -0.01556749,  0.00213151,  0.92394797};
    Eigen::Matrix3d rotation = q.toRotationMatrix();
    map2world_trans_.block<3,3>(0,0) = rotation;
    map2world_trans_(0,3) = 761678.903599;
    map2world_trans_(1,3) = 2529549.87151; 
    map2world_trans_(2,3) = 2.5507; 
    world2map_trans_ =  map2world_trans_.inverse();

    // downlidar2ins_(不准)
    downlidar2ins_ << 0 ,-1, 0, 0,
                    -1, 0, 0, 0,
                    0 , 0, -1, 0,
                     0, 0, 0, 1;
    // frontlidar2ins_
    frontlidar2ins_ <<  0,1,0,0,
                        -1,0,0,0,
                        0,0,1,0,
                        0,0,0,1;
}


void TFSubscriber::MsgCallback(const nav_msgs::OdometryConstPtr& odom) {
  buff_mutex_.lock();

  Eigen::Matrix4d ins2world = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q{odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z};
  Eigen::Matrix3d rotation = q.toRotationMatrix();
  ins2world.block<3,3>(0,0) = rotation;
  ins2world(0,3) = odom->pose.pose.position.x;
  ins2world(1,3) = odom->pose.pose.position.y;
  ins2world(2,3) = odom->pose.pose.position.z;
  Eigen::Matrix4d ins2map = world2map_trans_ * ins2world;
  
  // frontlidar2map
  Eigen::Matrix4d frontlidar2map = world2map_trans_ * ins2world * frontlidar2ins_;
  // std::cout << "world2map_trans_: " << world2map_trans_ << std::endl;
  // std::cout << "ins2world: " << ins2world << std::endl;
  // std::cout << "frontlidar2ins_ " << frontlidar2ins_ << std::endl;
  // downlidar2map(预留)
  Eigen::Matrix4d downlidar2map = world2map_trans_ * ins2world * downlidar2ins_;
  
  TFData tf_data(frontlidar2map, odom->header.stamp.toSec());
  // std::cout << "frontlidar2map: " << frontlidar2map << std::endl;

  if (new_tf_data_.size() > deque_size_ - 1) {
    new_tf_data_.pop_front();
    new_tf_data_.push_back(tf_data);
  } else {
    new_tf_data_.push_back(tf_data);
  }

  //   std::cout << " TFSubscriber::msg_callback new_tf_data_.size() = "
  //             << new_tf_data_.size() << std::endl;
  buff_mutex_.unlock();
}

void TFSubscriber::ParseData(std::deque<TFData>& tf_data_buff) {
    buff_mutex_.lock();

    if (new_tf_data_.size() > deque_size_ - 1) {
        tf_data_buff.insert(tf_data_buff.end(), new_tf_data_.begin(), new_tf_data_.end());
    } else{
    //   std::cout << "TFData deque size is " << new_tf_data_.size()
    //             << "less than 100, ParseData failed!" << std::endl;
    }
    buff_mutex_.unlock();
}

Eigen::Matrix4d TFSubscriber::FindNearestTf(const std::deque<TFData>& tf_data_buff, double time){
    // 找到最近时间的tf数据
    double min_time = 100, delt_time = 0.0;
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    TFData nearest_tf;
    for (auto tf : tf_data_buff) {
      delt_time = fabs(tf.time - time);
    //   std::cout << std::fixed << "delt_time: " << delt_time
    //             << ", tf.time:" << tf.time << " ,time:" << time << " s"
    //             << std::endl;
      if (delt_time < min_time) {
          nearest_tf = tf;
          min_time = delt_time;
      }
    }
    
    Eigen::Quaterniond q(nearest_tf.orientation.w, nearest_tf.orientation.x,
                         nearest_tf.orientation.y, nearest_tf.orientation.z);
    trans.block<3,3>(0,0) = q.matrix();
    trans(0,3) = nearest_tf.translation.x;
    trans(1,3) = nearest_tf.translation.y;
    trans(2,3) = nearest_tf.translation.z;
    trans(3,3) = 1;

    // std::cout << std::fixed << "min_time: " << min_time << " , point_time:" << time
    //           << " s \n"<< " trans: " << trans << std::endl;
    return trans;
}