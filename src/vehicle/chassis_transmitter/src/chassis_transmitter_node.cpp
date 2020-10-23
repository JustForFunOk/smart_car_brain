#include "tcp_client/tcp_client.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

::smart_car::TcpClient& tcp_client = ::smart_car::TcpClient::getSingleton();

void controlMsgCallback(const std_msgs::String::ConstPtr& _msg)
{
    // get msg from topic and send to tcp server
    if ( ::smart_car::TcpClient::kSuccess != tcp_client.write2TcpServer(_msg->data.c_str(), _msg->data.size()) )
    {
        printf("ERROR write to tcp server\n");
    }
}

int main(int argc, char **argv)
{
    // init ros parameter
    ros::init(argc, argv, "chassis_transmitter_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/chassis_control_cmd", 1000, controlMsgCallback);
    ros::Rate try_connect_rate_hz(1);  // try connect every 1 second
    ros::Rate check_callback_rate_hz(100);

    while (ros::ok())
    {
        // connect
        while (!tcp_client.isConnected())
        {
            auto connect_status = tcp_client.connect2TcpServer(std::string("192.168.0.106").c_str(), 5000);
            if (::smart_car::TcpClient::kSuccess == connect_status)
            {
                printf("Connect successful\n");
            }
            else
            {
                printf("Connect failed, error code: %d\n", connect_status);
                try_connect_rate_hz.sleep();
            }
        }

        ros::spinOnce();
        check_callback_rate_hz.sleep();
    }
    return 0;
}