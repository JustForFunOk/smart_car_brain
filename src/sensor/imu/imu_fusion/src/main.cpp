#include "imu_fusion_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_fusion_node");
    ::smartcar::sensor::imu::ImuFusionNode imu_fusion_node;
    imu_fusion_node.init();
    imu_fusion_node.run();
    return 0;
}