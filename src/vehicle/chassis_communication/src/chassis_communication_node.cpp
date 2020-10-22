#include "ros/ros.h"
#include "std_msgs/String.h"

#include <netinet/in.h>  // sockaddr_in
#include <netdb.h>  // hostent gethostbyname
#include <sys/socket.h>  // socket connect
#include <string.h>  // bzero

int main(int argc, char **argv)
{
    // init ros parameter
    ros::init(argc, argv, "chassis_communication_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/chassis_signal", 1000);
    ros::Rate try_connect_rate_hz(1);  // try connect every 1 second
    ros::Rate receive_data_rate_hz(10);

    // socket parameter
    bool is_connected = false;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    int sockfd = -1;
    int n = -1;
    int portno =  5000;
    const char* host_ip_addr = std::string("192.168.0.106").c_str();
    char buffer[256];

    while (ros::ok())
    {
        while (!is_connected)
        {
            // open socket
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0)
            {
                // ERROR, opening socket
                printf("ERROR, opening socket\n");
            }
            else
            {
                server = gethostbyname(host_ip_addr);
                if (server == NULL)
                {
                    // ERROR, no such host ip
                    printf("ERROR, no such host ip: %s\n", host_ip_addr);
                }
                else
                {
                    bzero((char *)&serv_addr, sizeof(serv_addr));
                    serv_addr.sin_family = AF_INET;
                    bcopy((char *)server->h_addr,
                          (char *)&serv_addr.sin_addr.s_addr,
                          server->h_length);
                    serv_addr.sin_port = htons(portno);

                    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
                    {
                        // ERROR connecting
                        printf("ERROR, can not connect, try connect again after 1s\n");
                        try_connect_rate_hz.sleep();
                    }
                    else
                    {
                        is_connected = true;
                    }
                }
            }
        }

        if (is_connected)
        {
            // get msg from tcp server and send to topic
            bzero(buffer,256);
            n = read(sockfd,buffer,255);  // block until receive data
            //TODO: how to exit, block, cannot exectue while (ros::ok())
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
            // ros::spinOnce();
            // receive_data_rate_hz.sleep();
        }
    }
    close(sockfd);
    return 0;
}