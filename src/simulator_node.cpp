#include <ros/ros.h>

#include <dataset_navigation_simulator/simulator.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dataset_nav_simulator_node");

    dataset_navigation_simulator::Simulator simulator;
    ros::spin();

    return 0;
}
