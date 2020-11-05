#include "tcp_client/tcp_client.h"

#include <stdio.h>  // printf
#include <netinet/in.h>  // sockaddr_in
#include <netdb.h>  // hostent gethostbyname
#include <sys/socket.h>  // socket connect
#include <string.h>  // bzero
#include <unistd.h>  //  close

namespace smart_car
{
namespace chassis
{

TcpClient::TcpClient() : sockfd_(-1), is_connected_(false)
{

}

TcpClient::~TcpClient()
{
    close(sockfd_);
}

TcpStatus TcpClient::connect2TcpServer(const char* _server_ip_addr, int _portno)
{
    // open socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0)
    {
        // ERROR, opening socket
        // printf("ERROR, opening socket\n");
        is_connected_ = false;
        return TcpStatus::kSocketOpenFailed;
    }

    struct hostent *server;
    server = gethostbyname(_server_ip_addr);
    if (server == NULL)
    {
        // ERROR, no such host ip
        // printf("ERROR, no such host name: %s\n", _server_ip_addr);
        is_connected_ = false;
        return TcpStatus::kTcpServerIpInvalid;
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
        is_connected_ = false;
        return TcpStatus::kConnectFailed;
    }
    else
    {
        is_connected_ = true;
        return TcpStatus::kSuccess;
    }
}

TcpStatus TcpClient::readFromTcpServer(char* _receive_data, size_t _len)
{
    if(read(sockfd_, _receive_data, _len) < 0)
    {
        return TcpStatus::kReadFromServerFailed;
    }
    else
    {
        return TcpStatus::kSuccess;
    }
}

TcpStatus TcpClient::write2TcpServer(const char* _send_data, size_t _len)
{
    if (write(sockfd_, _send_data, _len) < 0)
    {
        return TcpStatus::kWriteToServerFailed;
    }
    else
    {
        return TcpStatus::kSuccess;
    }
}

} // namespace chassis
} // namespace smart_car