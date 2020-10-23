# pragma once

#include <stddef.h>  // size_t

namespace smart_car
{
class TcpClient final
{
public:
    enum Status
    {
        kWriteToServerFailed = -5,
        kReadFromServerFailed = -4,
        kConnectFailed = -3,
        kTcpServerIpInvalid = -2,
        kSocketOpenFailed = -1,
        kSuccess = 0,
    };

public:
    static TcpClient& getSingleton()
    {
        static TcpClient tcp_client;
        return tcp_client;
    }
    /// read process is blocked
    int readFromTcpServer(char* _receive_data, size_t _len);
    int write2TcpServer(const char* _send_data, size_t _len);
    int connect2TcpServer(const char* server_ip_addr, int portno);
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
} // namespace smart_car