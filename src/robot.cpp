#include <dataset_navigation_simulator/robot.h>

using namespace dataset_navigation_simulator;

void
Robot::spinOnce()
{
    //for(Sensor::Ptr sensor: sensors_)
    for(int i = 0; i < sensors_.size(); i++)        
    {
        Sensor::Ptr sensor = sensors_[i];
        sensor->updateTransform(pose_);
        sensor->spinOnce(env_);        
    }
}
