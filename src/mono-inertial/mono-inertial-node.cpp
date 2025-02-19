#include "mono-inertial-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    SLAM_ = pSLAM;
    m_image_subscriber = this->create_subscription<ImageMsg>("camera",100,std::bind(&MonoInertialNode::GrabImage, this, std::placeholders::_1));
    
    rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
    subImu_ = this->create_subscription<ImuMsg>(
    "imu", imu_qos, std::bind(&MonoInertialNode::GrabImu, this, _1)
    );


    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_ ->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_); // Ensure thread safety
    imgBuf_.push(msg);

    // Debugging print
    // std::cout << "[DEBUG] GrabImage() called! "
    //           << "Timestamp: " << Utility::StampToSec(msg->header.stamp)
    //           << " Buffer size: " << imgBuf_.size() 
    //           << std::endl;
}





cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();

    // Debugging print
    // std::cout << "[DEBUG] GrabImu() called! "
    //           << "Timestamp: " << Utility::StampToSec(msg->header.stamp)
            //   << " Acceleration: (" << msg->linear_acceleration.x << ", "
            //   << msg->linear_acceleration.y << ", "
            //   << msg->linear_acceleration.z << ") "
            //   << " Gyro: (" << msg->angular_velocity.x << ", "
            //   << msg->angular_velocity.y << ", "
            //   << msg->angular_velocity.z << ")" 
            //   << std::endl;
}



void MonoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat im;
        double tIm = 0;

        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);

            // Debugging print
            // std::cout << "[DEBUG] SyncWithImu() Matching Image Timestamp: " << tIm << std::endl;

            if (tIm > Utility::StampToSec(imuBuf_.back()->header.stamp))
            {
                std::cout << "[DEBUG] Skipping: No matching IMU data for timestamp: " << tIm << std::endl;
                continue;
            }

            im = GetImage(imgBuf_.front());
            imgBuf_.pop();

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);

                    // Debugging print
                    // std::cout << "[DEBUG] IMU Data: t=" << t 
                    //           << " acc=(" << acc.x << ", " << acc.y << ", " << acc.z << ")"
                    //           << " gyr=(" << gyr.x << ", " << gyr.y << ", " << gyr.z << ")"
                    //           << std::endl;

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            // std::cout << "[DEBUG] Sending Image + " << vImuMeas.size() << " IMU samples to SLAM" << std::endl;

            // Check if ORB-SLAM3 starts tracking
            int trackingState = SLAM_->GetTrackingState();
            // std::cout << "[DEBUG] ORB-SLAM3 Tracking State: " << trackingState << std::endl;

            SLAM_->TrackMonocular(im, tIm, vImuMeas);
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
