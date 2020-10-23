#include "tcp_client/tcp_client.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    // init ros parameter
    ros::init(argc, argv, "chassis_receiver_node");
    ros::NodeHandle nh("~");

    std::string tcp_server_ip;
    if (! nh.getParam("tcp_server_ip", tcp_server_ip))
    {
        printf("no param named tcp_server_ip\n");
        return -1;
    }
    int portno;
    if (! nh.getParam("portno", portno))
    {
        printf("no param named portno\n");
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::String>("chassis_signal", 1000);

    ros::Rate try_connect_rate_hz(1);  // try connect every 1 second

    char rx_data[256];

    ::smart_car::TcpClient& tcp_client = ::smart_car::TcpClient::getSingleton();

    while (ros::ok())  //TODO: how to exit, read() blocked, cannot exectue while (ros::ok()) until receive msg
    {
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

        // get msg from tcp server and send to topic
        bzero(rx_data, sizeof(rx_data));
        if ( ::smart_car::TcpClient::kSuccess == tcp_client.readFromTcpServer(rx_data, sizeof(rx_data)) )
        {
            std_msgs::String msg;
            msg.data = rx_data;
            pub.publish(msg);
        }
        else
        {
            printf("ERROR reading from socket");
        }
    }
    return 0;
}