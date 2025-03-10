#include "mono-inertial-node.hpp"
#include "utility.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

// Max buffer size to prevent memory overflow
const int MAX_BUFFER_SIZE = 5000;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2"), SLAM_(pSLAM), syncThread_(new std::thread(&MonoInertialNode::SyncWithImu, this))
{
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera", 300, std::bind(&MonoInertialNode::GrabImage, this, _1));

    rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::KeepLast(3000)).best_effort();
    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", imu_qos, std::bind(&MonoInertialNode::GrabImu, this, _1));
}

MonoInertialNode::~MonoInertialNode()
{
    if (syncThread_ && syncThread_->joinable()) {
        syncThread_->join();
        delete syncThread_;
    }
}

void MonoInertialNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    double cam_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // std::cout << "📷 Camera Timestamp (sec.nanosec): " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;

    {
        std::lock_guard<std::mutex> lock(bufMutex_);

        if (!imuBuf_.empty()) {
            double closest_imu_time = 0;
            double min_diff = std::numeric_limits<double>::max();
            sensor_msgs::msg::Imu::SharedPtr best_imu_match = nullptr;

            // Copy queue into a temporary vector for iteration
            std::queue<sensor_msgs::msg::Imu::SharedPtr> temp_queue = imuBuf_;
            while (!temp_queue.empty()) {
                auto imu_msg = temp_queue.front();
                temp_queue.pop();

                double imu_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
                double diff = fabs(cam_time - imu_time);

                if (diff < min_diff) {
                    min_diff = diff;
                    closest_imu_time = imu_time;
                    best_imu_match = imu_msg;
                }
            }

            if (best_imu_match) {
                std::cout << "📡 Closest IMU Timestamp: " << closest_imu_time
                          << " (Difference: " << min_diff << " sec)" << std::endl;

                if (min_diff > 0.05) {
                    std::cerr << "⚠️  WARNING: IMU-Camera time difference is large: "
                              << min_diff << " sec" << std::endl;
                }
            }
        }

        imgBuf_.push(msg);

        // Prevent buffer overflow
        if (imgBuf_.size() > MAX_BUFFER_SIZE) {
            imgBuf_.pop();
        }
    }
}

void MonoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    std::cout << "📡 IMU Timestamp (sec.nanosec): " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;

    {
        std::lock_guard<std::mutex> lock(bufMutex_);
        imuBuf_.push(msg);

        // Debug: Print buffer size and first/last timestamps
        if (!imuBuf_.empty()) {
            double first_imu_time = imuBuf_.front()->header.stamp.sec + imuBuf_.front()->header.stamp.nanosec * 1e-9;
            double last_imu_time = imuBuf_.back()->header.stamp.sec + imuBuf_.back()->header.stamp.nanosec * 1e-9;
            std::cout << "🛑 IMU Buffer: Size=" << imuBuf_.size()
                      << " | First: " << first_imu_time
                      << " | Last: " << last_imu_time
                      << " | Current: " << imu_time << std::endl;
        }

        // Prevent buffer overflow
        if (imuBuf_.size() > MAX_BUFFER_SIZE) {
            imuBuf_.pop();
        }
    }
}


cv::Mat MonoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();
}

void MonoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    bool firstKeyframeInserted = false;

    while (rclcpp::ok()) {
        cv::Mat im;
        double tIm = 0;

        {
            std::lock_guard<std::mutex> lock(bufMutex_);

            if (!imgBuf_.empty() && !imuBuf_.empty()) {
                tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);

                if (tIm > Utility::StampToSec(imuBuf_.back()->header.stamp)) {
                    continue;
                }

                im = GetImage(imgBuf_.front());
                imgBuf_.pop();

                std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

                // Debug log
                // std::cout << "🟢 Processing Image at " << tIm << std::endl;

                // Extract IMU data closest to the image time
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm) {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(
                        imuBuf_.front()->linear_acceleration.x,
                        imuBuf_.front()->linear_acceleration.y,
                        imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(
                        imuBuf_.front()->angular_velocity.x,
                        imuBuf_.front()->angular_velocity.y,
                        imuBuf_.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }

                // Debug: Print how many IMU measurements were found
                // std::cout << "📡 IMU Measurements for this frame: " << vImuMeas.size() << std::endl;

                int trackingState = SLAM_->GetTrackingState();

                if (!firstKeyframeInserted && trackingState == 2) {
                    firstKeyframeInserted = true;
                }

                auto start = std::chrono::high_resolution_clock::now();
                SLAM_->TrackMonocular(im, tIm, vImuMeas);
                auto end = std::chrono::high_resolution_clock::now();

                // Debug: Show tracking state
                // std::cout << "🟣 ORB-SLAM Tracking State: " << trackingState << std::endl;

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
}
