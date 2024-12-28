#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ros2 run ros2_orb_slam3 rgbd /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/config/rgb-d/TUM3.yaml

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/camera/depth/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

}

RgbdSlamNode::~RgbdSlamNode()
{
    m_SLAM->Shutdown();
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f pose = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    if (!pose.matrix().isIdentity(1e-6)) // Check if pose is valid
    {
        PublishOdometry(pose.inverse(), msgRGB->header.stamp);
    }
}

void RgbdSlamNode::PublishOdometry(const Sophus::SE3f& pose, const rclcpp::Time& timestamp)
{    
    // Publish the initial pose as a static transform
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = timestamp;
    map_to_odom_tf.header.frame_id = "map";
    map_to_odom_tf.child_frame_id = "odom";

    map_to_odom_tf.transform.translation.x = -0.275132417678833;
    map_to_odom_tf.transform.translation.y = -0.740404486656189;
    map_to_odom_tf.transform.translation.z = 0.0;
    map_to_odom_tf.transform.rotation.x = 0.0;
    map_to_odom_tf.transform.rotation.y = 0.0;
    map_to_odom_tf.transform.rotation.z = -0.5331360401094174;
    map_to_odom_tf.transform.rotation.w = 0.8460295282887292;

    // Update and publish the map -> odom transform
    tf_broadcaster->sendTransform(map_to_odom_tf);

    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf rotation(pose.rotationMatrix());

    // RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Transilation: x=%.6f, y=%.6f, z=%.6f",
    //         translation.x(), translation.y(), translation.z());

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = translation.z();    // -z in pose is x in NAV2
    odom_msg.pose.pose.position.y = -translation.x();
    odom_msg.pose.pose.position.z = 0.0;  // Assuming 2D movement

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = -rotation.y();
    odom_msg.pose.pose.orientation.w = rotation.w();

    odom_pub->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = translation.z();
    tf_msg.transform.translation.y = -translation.x();
    tf_msg.transform.translation.z = 0.0;  // Assuming 2D movement
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = -rotation.y();
    tf_msg.transform.rotation.w = rotation.w();

    tf_broadcaster->sendTransform(tf_msg);
}
