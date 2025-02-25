#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "ros2_orb_slam3/utility.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    void PublishOdometry(const Sophus::SE3f& pose, const rclcpp::Time& timestamp);

    void SetInitialPose();

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose;

    bool initialize = true;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

#endif
