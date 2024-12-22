#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;

// Command to run the node

// ros2 run ros2_orb_slam3 ros2_mono /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/EuRoC.yaml

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(const string& voc_path, const string& sett_path)
        : Node("mono_node")
    {
        // Initialize the SLAM System
        SLAM = new ORB_SLAM3::System(voc_path, sett_path, ORB_SLAM3::System::MONOCULAR, true);

        // Create a subscription
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Mono node started. Subscribing to /camera/image_raw...");
    }

    ~ImageGrabber()
    {
        // Shutdown SLAM System
        SLAM->Shutdown();

        // Save KeyFrame Trajectory
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        delete SLAM;
    }

private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS2 Image Message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        int64_t nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

        double nanos_double = static_cast<double>(nanos);

        // Pass the image to ORB_SLAM3 for monocular tracking
        // SLAM->TrackMonocular(cv_ptr->image, nanos_double);

        SLAM->TrackMonocular(cv_ptr->image, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
        
        // Log the time in nanoseconds
        // RCLCPP_INFO(this->get_logger(), "Current Time (ns): %f", nanos_double);
    }

    ORB_SLAM3::System* SLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        cerr << endl << "Usage: ros2 run orb_slam3 mono_node path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    string voc_path = argv[1];
    string sett_path = argv[2];

    // Create the ROS2 node
    auto node = std::make_shared<ImageGrabber>(voc_path, sett_path);

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}





