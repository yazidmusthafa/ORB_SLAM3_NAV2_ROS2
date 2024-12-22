#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <mutex>
#include<queue>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/core/core.hpp>
#include "System.h"
#include "ImuTypes.h"

using namespace std;

// ros2 run ros2_orb_slam3 ros2_mono_inertial_node /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin /home/yazidmusthafa/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular-Inertial/gazeboCam.yaml


class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(const string& voc_path, const string& sett_path)
        : Node("mono_inertial_node")
    {
        // Initialize the SLAM System
        SLAM = new ORB_SLAM3::System(voc_path, sett_path, ORB_SLAM3::System::IMU_MONOCULAR, true);

        // Create a subscription
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 100,
            std::bind(&ImageGrabber::GrabIMU, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Mono node started. Subscribing to /camera/image_raw...");

        thread_ = std::thread(&ImageGrabber::SyncWithImu, this);
    }

    ~ImageGrabber()
    {   
        if (thread_.joinable())
        {
            thread_.join();
        }
        // Shutdown SLAM System
        SLAM->Shutdown();

        // Save KeyFrame Trajectory
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        delete SLAM;
    }

private:

    void SyncWithImu()
    {
        while (rclcpp::ok())
        {
            cv::Mat img;
            double tIm = 0;
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

            // Step 1: Get the latest image from the buffer
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (!imageBuf.empty())
                {
                    // Get image and timestamp
                    img = GetImage(imageBuf.front());
                    tIm = imageBuf.front()->header.stamp.sec +
                        imageBuf.front()->header.stamp.nanosec * 1e-9;
                    imageBuf.pop();
                }
            }

            if (img.empty())
            {
                // If no image is available, sleep briefly and continue
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Step 2: Get all IMU measurements up to the image timestamp
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                while (!imuBuf.empty() &&
                    imuBuf.front()->header.stamp.sec +
                            imuBuf.front()->header.stamp.nanosec * 1e-9 <= tIm)
                {
                    double t = imuBuf.front()->header.stamp.sec +
                            imuBuf.front()->header.stamp.nanosec * 1e-9;

                    cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
                                    imuBuf.front()->linear_acceleration.y,
                                    imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
                                    imuBuf.front()->angular_velocity.y,
                                    imuBuf.front()->angular_velocity.z);

                    vImuMeas.emplace_back(acc, gyr, t);
                    imuBuf.pop();
                }
            }

            // Step 3: Convert image to grayscale and apply CLAHE
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            mClahe->apply(gray, img);

            // Step 4: Track the frame using the SLAM system
            SLAM->TrackMonocular(img, tIm, vImuMeas);

            // Step 5: Optional: Visualize the processed image
            cv::imshow("Equalized Image", img);
            cv::waitKey(1);

            // Step 6: Sleep briefly to avoid busy looping
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }


    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if(!imageBuf.empty())
            imageBuf.pop();
        imageBuf.push(msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
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
        }

        return cv_ptr->image;
    }

    void GrabIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Lock the IMU buffer
        // std::unique_lock<std::mutex> lock(mutex_);
        std::lock_guard<std::mutex> lock(imu_mutex_);

        imuBuf.push(msg);
        
    }

    ORB_SLAM3::System* SLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> imageBuf;
    std::mutex image_mutex_;
    std::mutex imu_mutex_;
    std::thread thread_;

    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

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
