#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <unordered_map>
#include <thread>

#include <ros/ros.h>
#include <dataset_navigation_simulator/robot.h>
#include <dataset_navigation_simulator/navigator.h>
#include <tf/transform_broadcaster.h>

namespace dataset_navigation_simulator
{

    class Simulator
    {
    public:
        Simulator();

        ~Simulator();                

        // TODO : add pause capability through a service
        // void pause();

    private:
        EnvironmentModel::Ptr env_;
        std::unordered_map<std::string, Robot::Ptr> robots_;
        std::unordered_map<std::string, Navigator::Ptr> navigators_;

        octomap::OcTree* octree_;

        volatile bool paused_;
        std::shared_ptr<std::thread> main_thread_;
        std::shared_ptr<std::thread> tf_thread_;

        // parameters
        std::string fixed_frame_;        
        
        ros::NodeHandle* nh_;
        ros::NodeHandle* private_nh_;
        std::unordered_map<std::string, ros::Publisher> sensor_msg_pubs_;
        std::unordered_map<std::string, ros::Subscriber> nav_cmd_subs_;
        tf::TransformBroadcaster tbr_;
                
        void setUp();

        void run();

        void publish_tf();
    };
    
} // ~namespace

#endif
