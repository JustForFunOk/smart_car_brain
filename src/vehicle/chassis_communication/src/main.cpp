#include "chassis_communication_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis_transmitter_node");
    ::smart_car::chassis::ChassisCommunicationNode chassis_node;
    chassis_node.init();
    chassis_node.run();
    return 0;
}