#include <dataset_navigation_simulator/util.h>
#include <tf/transform_datatypes.h>

namespace dataset_navigation_simulator
{

    tf::Transform computeTransform(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf::Transform t;
        t.setOrigin(tf::Vector3(x,y,z));
        t.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));
        return t;
    }
}
