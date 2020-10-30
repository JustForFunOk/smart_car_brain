#include "tcp_client/tcp_client.h"
#include "chassis_communication_node.h"

namespace smart_car
{
namespace chassis
{

const uint32_t kMultiThreadNum = 2;  // transmit thread and receive thread
// Data
const uint8_t kTransmitDataLength = 18;  // keep update
// Accel
const double kGravityAccelG = 9.794;  // shanghai
const uint8_t kAccelSensorRange = 4; // +-4g
const double kAccelResolution = kGravityAccelG * kAccelSensorRange / 32768.0;
// Gyro
const uint16_t kGyroSensorRange = 250; // +-250 degrees per second
const double kGyroResolution = kGyroSensorRange / 32768.0;

::smart_car::TcpClient& tcp_client = ::smart_car::TcpClient::getSingleton();

void decodeAccelData(const uint8_t* _start_bit, double& _accel_x, double& _accel_y, double& _accel_z)
{
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    memcpy(&raw_accel_x, _start_bit, sizeof(int16_t));
    memcpy(&raw_accel_y, _start_bit+sizeof(int16_t), sizeof(int16_t));
    memcpy(&raw_accel_z, _start_bit+sizeof(int16_t)*2, sizeof(int16_t));
    // decode according to sensor config
    _accel_x = raw_accel_x * kAccelResolution;
    _accel_y = raw_accel_y * kAccelResolution;
    _accel_z = raw_accel_z * kAccelResolution;
}

void decodeGyroData(const uint8_t* _start_bit, double& _gyro_x, double& _gyro_y, double& _gyro_z)
{
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    memcpy(&raw_gyro_x, _start_bit, sizeof(int16_t));
    memcpy(&raw_gyro_y, _start_bit+sizeof(int16_t), sizeof(int16_t));
    memcpy(&raw_gyro_z, _start_bit+sizeof(int16_t)*2, sizeof(int16_t));
    // decode according to sensor config
    _gyro_x = raw_gyro_x * kGyroResolution;
    _gyro_y = raw_gyro_y * kGyroResolution;
    _gyro_z = raw_gyro_z * kGyroResolution;
}

void decodeMagnetData(const uint8_t* _start_bit, double& _magnet_x, double& _magnet_y, double& _magnet_z)
{
    // magnet has no data
    _magnet_x = 0;
    _magnet_y = 0;
    _magnet_z = 0;
}

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
        // raw chassis data
        chassis_communication::ChassisRawData raw_chassis_data;
        raw_chassis_data.data = std::move(std::vector<uint8_t>(rx_data, rx_data+kTransmitDataLength));
        raw_data_pub_.publish(raw_chassis_data);

        // decoded chassis data
        chassis_communication::DecodedChassisData decoded_chassis_data;
        decodeChassisData(raw_chassis_data, decoded_chassis_data);
        decoded_data_pub_.publish(decoded_chassis_data);
    }
    else
    {
        printf("ERROR reading from socket\n");
    }
}

void ChassisCommunicationNode::decodeChassisData(const chassis_communication::ChassisRawData& _raw_chassis_data,
                                                 chassis_communication::DecodedChassisData& _decoded_chassis_data)
{
    decodeAccelData(&_raw_chassis_data.data[0], _decoded_chassis_data.accel.x, _decoded_chassis_data.accel.y, _decoded_chassis_data.accel.z);
    decodeGyroData(&_raw_chassis_data.data[6], _decoded_chassis_data.gyro.x, _decoded_chassis_data.gyro.y, _decoded_chassis_data.gyro.z);
    decodeMagnetData(&_raw_chassis_data.data[12], _decoded_chassis_data.magnet.x, _decoded_chassis_data.magnet.y, _decoded_chassis_data.magnet.z);
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
    raw_data_pub_ = nh.advertise<chassis_communication::ChassisRawData>("raw_chassis_signal", 10);
    decoded_data_pub_ = nh.advertise<chassis_communication::DecodedChassisData>("decoded_chassis_signal", 10);

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