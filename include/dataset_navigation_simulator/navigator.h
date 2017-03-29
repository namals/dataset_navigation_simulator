#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include <actionlib_msgs/GoalStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <dataset_navigation_simulator/robot.h>

#include <memory>

namespace dataset_navigation_simulator
{

    class Navigator
    {
    public:
        typedef std::shared_ptr<Navigator> Ptr;
        typedef std::shared_ptr<const Navigator> ConstPtr;
        
        Navigator(Robot::Ptr robot)            
            : robot_(robot)
            , d_len_thresh_(0.3)
            , d_theta_thresh_(0.17)
            , look_ahead_dist_(d_len_thresh_)
            , map_service_name_("octomap_binary")            
        {
            goal_status_.status = actionlib_msgs::GoalStatus::PENDING; 
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

        void setGoalStatus(actionlib_msgs::GoalStatus& status)
        {
            goal_status_ = status;
        }

        actionlib_msgs::GoalStatus getGoalStatus()
        {
            return goal_status_;
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

        actionlib_msgs::GoalStatus goal_status_;

        tf::Transform curr_pose_;

        int plan_idx_;

        std::string map_service_name_;

        ros::ServiceClient map_service_client_;
    }; 
    
} //~namespace

#endif
