#include "tcp_client/tcp_client.h"

#include <stdio.h>
#include <netinet/in.h>  // sockaddr_in
#include <netdb.h>  // hostent gethostbyname
#include <sys/socket.h>  // socket connect
#include <string.h>  // bzero
#include <unistd.h>  //  close

namespace smart_car
{


// int n = -1;

// const char* host_ip_addr = std::string("192.168.0.106").c_str();


TcpClient::TcpClient() : sockfd_(-1)
{

}

TcpClient::~TcpClient()
{
    close(sockfd_);
}

int TcpClient::connect2TcpServer(const char* _server_ip_addr, int _portno)
{
    // open socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0)
    {
        // ERROR, opening socket
        // printf("ERROR, opening socket\n");
        return kSocketOpenFailed;
    }

    struct hostent *server;
    server = gethostbyname(_server_ip_addr);
    if (server == NULL)
    {
        // ERROR, no such host ip
        // printf("ERROR, no such host name: %s\n", _server_ip_addr);
        return kTcpServerIpInvalid;
    }

    struct sockaddr_in serv_addr;
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(_portno);
    if (connect(sockfd_, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        // ERROR connecting
        // printf("ERROR, can not connect, try connect again after 1s\n");
        return kConnectFailed;
    }
    else
    {
        return kSuccess;
    }
}

int TcpClient::readFromTcpServer()
{
    if(read(sockfd_,buffer_,sizeof(buffer_)) < 0)
    {
        return kReadFromServerFailed;
    }
    else
    {
        return kSuccess;
    }
}

int TcpClient::write2TcpServer(const char* _send_data, size_t _len)
{
    if (write(sockfd_, _send_data, _len) < 0)
    {
        return kWriteToServerFailed;
    }
    else
    {
        return kSuccess;
    }
}

} // namespace smart_car
