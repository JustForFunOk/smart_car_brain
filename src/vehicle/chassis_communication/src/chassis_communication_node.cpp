#include "tcp_client/tcp_client.h"
#include "chassis_communication_node.h"
#include "chassis_communication/ChassisRawData.h"

namespace smart_car
{
namespace chassis
{

const uint32_t kMultiThreadNum = 2;  // transmit thread and receive thread

const uint8_t kTransmitDataLength = 18;  // keep update

::smart_car::TcpClient& tcp_client = ::smart_car::TcpClient::getSingleton();

void ChassisCommunicationNode::transmitMsgCallback(const std_msgs::String::ConstPtr& _msg)
{
    printf("get control cmd\n");
    // get msg from topic and send to tcp server
    if ( ::smart_car::TcpClient::kSuccess != tcp_client.write2TcpServer(_msg->data.c_str(), _msg->data.size()) )
    {
        printf("ERROR write to tcp server\n");
    }
}

void ChassisCommunicationNode::receiveMsgCallback(const ros::TimerEvent&)
{
    printf("wait for chassis msg\n");
    char rx_data[kTransmitDataLength];
    // get msg from tcp server and send to topic
    bzero(rx_data, kTransmitDataLength);
    if ( ::smart_car::TcpClient::kSuccess == tcp_client.readFromTcpServer(rx_data, kTransmitDataLength) )
    {
        printf("receive chassis msg\n");
        chassis_communication::ChassisRawData chassis_raw_data;
        chassis_raw_data.data = std::move(std::vector<uint8_t>(rx_data, rx_data+kTransmitDataLength));
        pub_.publish(chassis_raw_data);
    }
    else
    {
        printf("ERROR reading from socket\n");
    }
}

void ChassisCommunicationNode::init()
{
    // init ros parameter
    ros::NodeHandle nh("~");

    std::string tcp_server_ip;
    if (! nh.getParam("tcp_server_ip", tcp_server_ip))
    {
        printf("no param named tcp_server_ip\n");
    }
    int portno;
    if (! nh.getParam("portno", portno))
    {
        printf("no param named portno\n");
    }

    timer_ = nh.createTimer(ros::Duration(0.1), &ChassisCommunicationNode::receiveMsgCallback, this);  // readFromTcpServer is blocked
    pub_ = nh.advertise<chassis_communication::ChassisRawData>("chassis_raw_signal", 10);

    sub_ = nh.subscribe("/chassis_control_cmd", 10, &ChassisCommunicationNode::transmitMsgCallback, this);

    ros::Rate try_connect_rate_hz(1);  // try connect every 1 second

    // connect
    while (!tcp_client.isConnected())
    {
        auto connect_status = tcp_client.connect2TcpServer(tcp_server_ip.c_str(), portno);
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
}

void ChassisCommunicationNode::run()
{
    ros::MultiThreadedSpinner spinner(kMultiThreadNum);
    spinner.spin();
}

} // namespace chassis
} // namespace smart_car