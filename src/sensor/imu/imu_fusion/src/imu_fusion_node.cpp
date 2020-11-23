#include "imu_fusion_node.h"
#include "MahonyAHRS.h"

#include <sensor_msgs/Imu.h>

namespace smartcar
{
namespace sensor
{
namespace imu
{

ImuFusionNode::ImuFusionNode(/* args */)
{
}

ImuFusionNode::~ImuFusionNode()
{
}

void ImuFusionNode::init()
{
    ros::NodeHandle nh("~");

    std::string topic_in_9axis_raw;
    if (! nh.getParam("topic_in_9axis_raw", topic_in_9axis_raw))
    {
        printf("no param named topic_in_9axis_raw\n");
    }

    std::string topic_out_9axis_fusion;
    if (! nh.getParam("topic_out_9axis_fusion", topic_out_9axis_fusion))
    {
        printf("no param named topic_out_9axis_fusion\n");
    }

    pub_fusion_data_ = nh.advertise<sensor_msgs::Imu>(topic_out_9axis_fusion, 10);

    sub_raw_data_ = nh.subscribe(topic_in_9axis_raw, 10, &ImuFusionNode::receiveImuRawMsgCallback, this);
}

void ImuFusionNode::run()
{
    ros::spin();
}

void ImuFusionNode::receiveImuRawMsgCallback(chassis_communication::DecodedChassisData::ConstPtr _decoded_data)
{
    MahonyAHRSupdate(_decoded_data->gyro.x, _decoded_data->gyro.y, _decoded_data->gyro.z,
                     _decoded_data->accel.x, _decoded_data->accel.y, _decoded_data->accel.z,
                     _decoded_data->magnet.x, _decoded_data->magnet.y, _decoded_data->magnet.z);

    sensor_msgs::Imu fusion_data;
    // header
    fusion_data.header.stamp = ros::Time::now();
    fusion_data.header.frame_id = "ego_car";
    // data
    fusion_data.orientation.x = q0;
    fusion_data.orientation.y = q1;
    fusion_data.orientation.z = q2;
    fusion_data.orientation.w = q3;

    fusion_data.angular_velocity = _decoded_data->gyro;

    fusion_data.linear_acceleration = _decoded_data->accel;

    pub_fusion_data_.publish(fusion_data);
}

} // namespace imu
} // namespace sensor  
} // namespace smartcar