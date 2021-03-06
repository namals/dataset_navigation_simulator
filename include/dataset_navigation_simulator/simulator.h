#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <ros/ros.h>
#include <dataset_navigation_simulator/robot.h>
#include <dataset_navigation_simulator/navigator.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

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
        boost::unordered_map<std::string, Robot::Ptr> robots_;
        boost::unordered_map<std::string, Navigator::Ptr> navigators_;

        octomap::OcTree* octree_;

        volatile bool paused_;
        boost::shared_ptr<boost::thread> main_thread_;
        boost::shared_ptr<boost::thread> tf_thread_;

        // parameters
        std::string fixed_frame_;

        ros::NodeHandle* nh_;
        ros::NodeHandle* private_nh_;
        ros::Subscriber pause_resume_sub_;        
        boost::unordered_map<std::string, ros::Subscriber> nav_cmd_subs_;
        boost::unordered_map<std::string, ros::Publisher> nav_status_pubs_;
        boost::unordered_map<std::string, ros::Publisher> sensor_msg_pubs_;
        boost::unordered_map<std::string, ros::Publisher> sensor_fov_vis_pubs_;
        tf::TransformBroadcaster tbr_;

        void setUp();

        void run();

        void publish_tf();

        void pauseResumeCb(std_msgs::Bool msg);

        void publishSensorFOV(ros::Publisher& pub, Sensor::Ptr spec);
    };

} // ~namespace

#endif
