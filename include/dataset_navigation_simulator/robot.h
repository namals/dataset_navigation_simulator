#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <cstring>

#include <dataset_navigation_simulator/environment_model.h>
#include <dataset_navigation_simulator/sensor.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

namespace dataset_navigation_simulator
{

    class Robot
    {
    public:
        typedef boost::shared_ptr<Robot> Ptr;
        typedef boost::shared_ptr<const Robot> ConstPtr;
        
        Robot(std::string robot_frame,
              std::string world_frame,              
              const tf::Transform& pose,
              double radius,
              EnvironmentModel::Ptr env)              
            : robot_frame_(robot_frame)
            , world_frame_(world_frame)
            , pose_(pose)
            , radius_(radius)
            , env_(env)           
        {
        }

        void setRobotFrameName(std::string robot_frame)
        {
            robot_frame_ = robot_frame;
        }

        std::string getRobotFrameName()
        {
            return robot_frame_;
        }

        void setWorldFrameName(std::string world_frame)
        {
            world_frame_ = world_frame;
        }

        std::string getWorldFrameName()
        {
            return world_frame_;
        }

        void setRobotPose(const tf::Transform& pose)
        {
            mutex_.lock();
            pose_ = pose;
            mutex_.unlock();
        }

        tf::Transform getRobotPose()
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            return pose_;
        }

        void setRobotRadius(double radius)
        {
            radius_ = radius;
        }

        double getRobotRadius()
        {
            return radius_;
        }

        void addSensor(Sensor::Ptr& sensor)
        {
            sensors_.push_back(sensor);
        }

        std::vector<Sensor::Ptr> getSensors()
        {
            return sensors_;
        }

        void setEnvironmentModel(EnvironmentModel::Ptr& env)
        {
            env_ = env;
        }

        EnvironmentModel::Ptr getEnvironmentModel()
        {
            env_;
        }
        
        void spinOnce();

    private:                
        std::string robot_frame_;

        std::string world_frame_;

        tf::Transform pose_;               /*!< Robot's pose w.r.t. map frame */

        double radius_;                    /*!< The radius of the robot */

        std::vector<Sensor::Ptr> sensors_;
        
        EnvironmentModel::Ptr env_;

        boost::mutex mutex_;
    };
    
} //~namespace

#endif
