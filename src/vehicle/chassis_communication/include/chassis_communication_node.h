# pragma once

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

private:
    ros::Timer timer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

} // namespace chassis
} // namespace smart_car
