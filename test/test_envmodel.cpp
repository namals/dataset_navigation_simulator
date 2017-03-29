#include <iostream>
#include <thread>

#include <tf/transform_broadcaster.h>

#include <dataset_navigation_simulator/environment_model.h>
#include <geometry_msgs/TransformStamped.h>

namespace dns = dataset_navigation_simulator;

using namespace std;
using std::cout;
using std::cin;

void publishTF(tf::StampedTransform& transformTF);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_envmodel");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");    

    // read the .bt file name
    std::string octomap_fname;
    if( !private_nh.getParam("octomap_fname", octomap_fname) )
    {
        ROS_FATAL("Octomap file name is not provided!");
        ros::shutdown();
        exit(0);
    }
    
    // read the octomap from .bt file
    octomap::OcTree* octree = new octomap::OcTree(octomap_fname);
    if( octree == NULL )
    {
        ROS_FATAL("Error in reading the octomap file!");
        ros::shutdown();
        exit(0);
    }

    ros::Publisher sensor_pub = private_nh.advertise<sensor_msgs::PointCloud2>("openni2_points", 1, true);

    dns::EnvironmentModel em(octree);

    // create sensor's transform    
    cout << "Reading sensor position : ";
    geometry_msgs::TransformStamped transform;    
    transform.header.frame_id = "map";
    transform.child_frame_id = "openni2_camera";
    cout << "x : ";
    cin >> transform.transform.translation.x;
    cout << "y : ";
    cin >> transform.transform.translation.y;
    cout << "z : ";
    cin >> transform.transform.translation.z;

    cout << "Reading sensor yaw : ";
    double yaw;
    cin >> yaw;
    yaw = yaw*M_PI/180.0;
    transform.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

    tf::StampedTransform transformTF;
    transform.header.stamp = ros::Time::now();
    tf::transformStampedMsgToTF(transform, transformTF);
    std::thread pubtf_thread(publishTF, std::ref(transformTF));

    dns::SensorSpec sensor_spec(0.5, 3.5, 1.01229, 0.785398);
    sensor_msgs::PointCloud2 sensor_msg;
    em.getSensorFrustum(transformTF, sensor_spec, sensor_msg);

    sensor_msg.header.stamp = transform.header.stamp;
    sensor_msg.header.frame_id = transform.child_frame_id;

    sensor_pub.publish(sensor_msg);
    
    ros::spin();

    // sleep(2);

    // cout << "Enter to exit";
    // char ch;
    // cin >> ch;

    // ros::shutdown();
    // pubtf_thread.join();

    return 0;
}

void publishTF(tf::StampedTransform& transformTF)
{
    static tf::TransformBroadcaster tb;
    
    ros::Rate r(20);
    while(ros::ok())
    {
        tb.sendTransform(transformTF);
        r.sleep();
    }
}
