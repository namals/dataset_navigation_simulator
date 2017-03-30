#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <dataset_navigation_simulator/robot.h>

#include <boost/shared_ptr.hpp>

namespace dataset_navigation_simulator
{
    struct NavigationStatus
    {
        enum Status
        {
            WAITING = 100,
            ACTIVE,            
            SUCCEEDED,
            ABORTED,
            INIT
        };

        char status;
    };

    class Navigator
    {
    public:
        typedef boost::shared_ptr<Navigator> Ptr;
        typedef boost::shared_ptr<const Navigator> ConstPtr;
        
        Navigator(Robot::Ptr robot)            
            : robot_(robot)
            , d_len_thresh_(0.3)
            , d_theta_thresh_(0.17)
            , look_ahead_dist_(d_len_thresh_)
            , map_service_name_("octomap_binary")            
        {
            nav_status_.status = NavigationStatus::INIT; 
        }

        void step();

        void setRobot(Robot::Ptr& robot)
        {
            robot_ = robot;
        }

        Robot::Ptr getRobot()
        {
            return robot_;
        }

        void setPlan(const nav_msgs::Path& plan)
        {
            plan_ = plan;
            plan_idx_ = 0;
        }

        nav_msgs::Path getPlan()
        {
            return plan_;
        }

        void setNavigationStatus(NavigationStatus& status)
        {
            nav_status_ = status;
        }

        NavigationStatus getNavigationStatus()
        {
            return nav_status_;
        }

        void setDeltaLengthThreshold(double dlen_thresh)
        {
            d_len_thresh_ = dlen_thresh;
        }

        double getDeltaLengthThreshold()
        {
            return d_len_thresh_;
        }

        void setDeltaThetaThreshold(double dtheta_thresh)
        {
            d_theta_thresh_ = dtheta_thresh;
        }

        double setDeltaThetaThreshold()
        {
            return d_theta_thresh_;
        }

        void setCurrentPose(tf::Transform& pose)
        {
            curr_pose_ = pose;
        }

        tf::Transform getCurrentPose()
        {
            return curr_pose_;
        }

        void setMapServiceName(std::string map_service_name, ros::NodeHandle& nh);

        std::string getMapServieName()
        {
            return map_service_name_;
        }

        void planCallback(const nav_msgs::Path::ConstPtr& path);
        
    private:
        Robot::Ptr robot_;
        
        double d_len_thresh_;

        double d_theta_thresh_;

        double look_ahead_dist_;
        
        nav_msgs::Path plan_;

        //actionlib_msgs::GoalStatus goal_status_;
        NavigationStatus nav_status_;

        tf::Transform curr_pose_;

        int plan_idx_;

        std::string map_service_name_;

        ros::ServiceClient map_service_client_;
    }; 
    
} //~namespace

#endif
