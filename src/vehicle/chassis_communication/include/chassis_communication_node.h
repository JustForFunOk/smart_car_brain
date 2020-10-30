# pragma once

#include "chassis_communication/ChassisRawData.h"
#include "chassis_communication/DecodedChassisData.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace smart_car
{
namespace chassis
{

class ChassisCommunicationNode
{
public:
    ChassisCommunicationNode() = default;
    ~ChassisCommunicationNode() = default;
    void init();
    void run();

private:
    void transmitMsgCallback(const std_msgs::String::ConstPtr& _msg);
    void receiveMsgCallback(const ros::TimerEvent&);
    void decodeChassisData(const chassis_communication::ChassisRawData& _raw_chassis_data,
                           chassis_communication::DecodedChassisData& _decoded_chassis_data);

private:
    ros::Timer timer_;
    ros::Publisher raw_data_pub_;
    ros::Publisher decoded_data_pub_;
    ros::Subscriber sub_;
};

} // namespace chassis
} // namespace smart_car
