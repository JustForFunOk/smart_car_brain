#include "ros/ros.h"
#include "std_msgs/String.h"

#include <netinet/in.h>  // sockaddr_in
#include <netdb.h>  // hostent gethostbyname
#include <sys/socket.h>  // socket connect
#include <string.h>  // bzero

void controlMsgCallback(const std_msgs::String::ConstPtr& msg)
{
    printf("enter callback\n");
}

int main(int argc, char **argv)
{
    // init ros parameter
    ros::init(argc, argv, "chassis_communication_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/chassis_signal", 1000);
    ros::Subscriber sub = nh.subscribe("/control_cmd", 1000, controlMsgCallback);
    ros::Rate try_connect_rate_hz(1);  // try connect every 1 second
    ros::Rate receive_data_rate_hz(10);

    // socket parameter
    bool is_connected = false;


    while (ros::ok() && !is_connected)
    {
        try_connect_rate_hz.sleep();
    }

    while (ros::ok())  //TODO: how to exit, read() blocked, cannot exectue while (ros::ok()) until receive msg
    {
        // get msg from tcp server and send to topic
        bzero(buffer,256);
        n = read(sockfd,buffer,255);  // blocked until receive data
        if (n < 0)
        {
            printf("ERROR reading from socket");
        }
        else
        {
            std_msgs::String msg;
            msg.data = buffer;
            pub.publish(msg);
        }

        printf("loop once\n");
        ros::spinOnce();
        receive_data_rate_hz.sleep();
    }



    return 0;
}