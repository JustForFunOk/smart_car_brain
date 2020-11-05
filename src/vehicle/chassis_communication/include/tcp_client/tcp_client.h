# pragma once

#include <stddef.h>  // size_t

namespace smart_car
{
namespace chassis
{

enum class TcpStatus
{
    kWriteToServerFailed = -5,
    kReadFromServerFailed = -4,
    kConnectFailed = -3,
    kTcpServerIpInvalid = -2,
    kSocketOpenFailed = -1,
    kSuccess = 0,
};

class TcpClient final
{
public:
    static TcpClient& getSingleton()
    {
        static TcpClient tcp_client;
        return tcp_client;
    }
    /// read process is blocked
    TcpStatus readFromTcpServer(char* _receive_data, size_t _len);
    TcpStatus write2TcpServer(const char* _send_data, size_t _len);
    TcpStatus connect2TcpServer(const char* server_ip_addr, int portno);
    inline bool isConnected() {return is_connected_;}

private:
    TcpClient();
    ~TcpClient();
    TcpClient(const TcpClient&);
    TcpClient& operator=(const TcpClient&);

private:
    int sockfd_;
    bool is_connected_;
};

} // namespace chassis
} // namespace smart_car