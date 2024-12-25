#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
        PublishOdometry(pose, msgRGB->header.stamp);
    }
}

void RgbdSlamNode::PublishOdometry(const Sophus::SE3f& pose, const rclcpp::Time& timestamp)
{
    if (initialize == true){
        
        geometry_msgs::msg::PoseWithCovarianceStamped init_pose;       // Try top make it automatically taking the transform of the robot base link
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.position.x = -0.275132417678833;
        init_pose.pose.pose.position.y = -0.740404486656189;
        init_pose.pose.pose.position.z = 0.0;
        init_pose.pose.pose.orientation.x = 0.0;
        init_pose.pose.pose.orientation.y = 0.0;
        init_pose.pose.pose.orientation.z = -0.5331360401094174;
        init_pose.pose.pose.orientation.w = 0.8460295282887292;
        initial_pose->publish(init_pose);

        initialize = false;
    }

    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf rotation(pose.rotationMatrix());

    // Extract yaw from the quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()))
        .getRPY(roll, yaw, pitch);

    // Create a new quaternion with only yaw
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, yaw);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = -translation.z();    // -z in pose is x in NAV2 for some reason
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = 0.0;  // Assuming 2D movement

    odom_msg.pose.pose.orientation.x = yaw_quat.x();
    odom_msg.pose.pose.orientation.y = yaw_quat.y();
    odom_msg.pose.pose.orientation.z = yaw_quat.z();
    odom_msg.pose.pose.orientation.w = yaw_quat.w();

    odom_pub->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = -translation.z();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = 0.0;  // Assuming 2D movement
    tf_msg.transform.rotation.x = yaw_quat.x();
    tf_msg.transform.rotation.y = yaw_quat.y();
    tf_msg.transform.rotation.z = yaw_quat.z();
    tf_msg.transform.rotation.w = yaw_quat.w();

    tf_broadcaster->sendTransform(tf_msg);
}
