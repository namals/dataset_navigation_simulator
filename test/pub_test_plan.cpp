#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <fstream>
#include <iostream>
#include <string>

#include <boost/algorithm/string.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pub_test_plan");

    // read the plan from parameter server
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("/dataset_nav_simulator_node/base_link1/plan", 1);
    //ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("/plan", 1);

    sleep(1);

    std::string path_file;
    if( !private_nh.getParam("path_file", path_file) )
    {
        ROS_FATAL("File containing planned path is not provided!");
        exit(0);
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    
    std::ifstream pfile(path_file);
    // read the first line
    std::string q_str;
    //pfile >> q_str;
    std::getline(pfile, q_str);
    int ln = 1;
    while(pfile.good())
    {
        if( q_str.compare("") == 0 )
        {
            std::getline(pfile, q_str);            
            continue;
        }

        std::cout << q_str << std::endl;
        std::vector<std::string> vals;
        boost::split(vals, q_str, boost::is_any_of(","));
        if( vals.size() != 6 )
        {
            std::cout << "Something is wrong in the inputline!" << std::endl;
            return -1;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.seq = ln++;
        pose.header.frame_id = "map";        
        boost::trim(vals[0]);
        pose.pose.position.x = std::stod(vals[0]);
        boost::trim(vals[1]);
        pose.pose.position.y = std::stod(vals[1]);
        boost::trim(vals[2]);
        pose.pose.position.z = std::stod(vals[2]);
        double r, p, y;
        boost::trim(vals[3]);
        r = angles::from_degrees(std::stod(vals[3]));
        boost::trim(vals[4]);
        p = angles::from_degrees(std::stod(vals[4]));
        boost::trim(vals[5]);
        y = angles::from_degrees(std::stod(vals[5]));
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);

        path.poses.push_back(pose);
        
        // read the next line
        //pfile >> q_str;
        std::getline(pfile, q_str);
    }    

    // now publish the path
    plan_pub.publish(path);

    ros::spin();
    
    return 0;
}
