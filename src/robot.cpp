#include <dataset_navigation_simulator/robot.h>

using namespace dataset_navigation_simulator;

void
Robot::spinOnce()
{
    for(Sensor::Ptr sensor: sensors_)
    {
        sensor->updateTransform(pose_);
        sensor->spinOnce(env_);        
    }
}
