# pragma once

#include "chassis_communication/DecodedChassisData.h"

#include <ros/ros.h>

namespace smartcar
{
namespace sensor
{
namespace imu
{

class ImuFusionNode
{
public:
    ImuFusionNode(/* args */);
    ~ImuFusionNode();
    void init();
    void run();

private:
    void receiveImuRawMsgCallback(chassis_communication::DecodedChassisData::ConstPtr);

private:
    ros::Publisher pub_fusion_data_;
    ros::Subscriber sub_raw_data_;
};

} // namespace imu
} // namespace sensor  
} // namespace smartcar