#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);

    ~MonocularInertialNode();

private:

    void GrabImu(const ImuMsg::SharedPtr msg);

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void SyncWithImu();
    
    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;

    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;
};

#endif
