#include <dataset_navigation_simulator/simulator.h>
#include <angles/angles.h>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

using namespace dataset_navigation_simulator;

Simulator::Simulator()
    : nh_(new ros::NodeHandle())
    , private_nh_(new ros::NodeHandle("~"))
{
    
    setUp();

    // start the thread
    main_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind(&Simulator::run, this) ) );
    tf_thread_ = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind(&Simulator::publish_tf, this) ) );
}

Simulator::~Simulator()
{
    delete nh_;
    delete private_nh_;
    if( octree_ )
        delete octree_;
}

void
Simulator::run()
{
    while(!paused_)
    {
        //for(auto robot_nav : navigators_ )        
        for(boost::unordered_map<std::string, Navigator::Ptr>::iterator nit = navigators_.begin(); nit != navigators_.end(); nit++)
        {
            std::pair<std::string, Navigator::Ptr> robot_nav = *nit;
            robot_nav.second->step();
        }

        usleep(100000);  // wait 100 ms so that all TF's are adjusted for current pose of robots
        
        // go through each robot and spin each robot once
        for(boost::unordered_map<std::string, Robot::Ptr>::iterator rit = robots_.begin(); rit != robots_.end(); rit++)
        {
            std::pair<std::string, Robot::Ptr> rdata = *rit;
            rdata.second->spinOnce();

            // // don't publish if navigator in SUCCEEDED state
            NavigationStatus status = navigators_.at(rdata.second->getRobotFrameName())->getNavigationStatus();

            // now publish navigator status
            nav_status_pubs_.at(rdata.second->getRobotFrameName()).publish(status);
            
            if( status.status == NavigationStatus::WAITING )
                continue;
            
            // get all sensor data of this robot
            std::vector<Sensor::Ptr> sensors = rdata.second->getSensors();
            for(std::vector<Sensor::Ptr>::iterator sit = sensors.begin(); sit != sensors.end(); sit++)
            {
                Sensor::Ptr sensor = *sit;
                sensor_msg_pubs_.at(sensor->getSensorFQName()).publish(sensor->getCurrentSensorData());
            }            
        }        
        
        usleep(900000); // sleep for 900ms
    }
}

void
Simulator::setUp()
{
    // initialize the environment model
    std::string envmodel_octomap_fname;
    if( !private_nh_->getParam("envmodel_fname", envmodel_octomap_fname) )
    {
        ROS_FATAL("Environment model file name is not provided!");
        ros::shutdown();
        exit(0);
    }
    octree_ = new octomap::OcTree(envmodel_octomap_fname);
    env_ = EnvironmentModel::Ptr(new EnvironmentModel(octree_));
    
    // read the names of the robots 
    std::vector<std::string> robot_names;
    private_nh_->getParam("robot_names", robot_names);
    
    private_nh_->param<std::string>("fixed_frame", fixed_frame_, "map");

    // now for each robot, read the sensor and navigator parameters
    for(std::vector<std::string>::iterator rnit = robot_names.begin(); rnit != robot_names.end(); rnit++)
    {
        std::string robot_name = *rnit;
        double pos_x, pos_y, pos_z, roll, pitch, yaw;
        private_nh_->param(robot_name + "/pos_x", pos_x, 0.0);
        private_nh_->param(robot_name + "/pos_y", pos_y, 0.0);
        private_nh_->param(robot_name + "/pos_z", pos_z, 0.0);
        private_nh_->param(robot_name + "/roll", roll, 0.0);
        private_nh_->param(robot_name + "/pitch", pitch, 0.0);
        private_nh_->param(robot_name + "/yaw", yaw, 0.0);
        tf::Transform t_r_m = computeTransform(pos_x, pos_y, pos_z, angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
        double rradius;
        private_nh_->param<double>(robot_name + "/radius", rradius, 0.3);
        Robot::Ptr robot(new Robot(robot_name, fixed_frame_, t_r_m, rradius, env_));        
        std::vector<std::string> sensor_names;
        private_nh_->getParam(robot_name + "/sensor_names", sensor_names);
        
        for(std::vector<std::string>::iterator snit = sensor_names.begin(); snit != sensor_names.end(); snit++)
        {
            std::string sensor_name = *snit;
            std::map<std::string,double> sensor_details;
            private_nh_->getParam(robot_name + "/" + sensor_name, sensor_details);
            
            double rmin, rmax, hfov, vfov, rres;
            if( sensor_details.find("r_min") != sensor_details.end() )
                rmin = sensor_details["r_min"];
            else
                rmin = 0.5;
            
            if( sensor_details.find("r_max") != sensor_details.end() )
                rmax  = sensor_details["r_max"];
            else
                rmax = 5.0;

            if( sensor_details.find("hor_fov") != sensor_details.end() )
                hfov = sensor_details["hor_fov"];
            else
                hfov = 1.01;

            if( sensor_details.find("ver_fov") != sensor_details.end() )
                vfov = sensor_details["ver_fov"];
            else
                vfov = 0.79;

            if( sensor_details.find("ray_res") != sensor_details.end() )
                rres = sensor_details["ray_res"];
            else
                rres = 0.028571429;

            // read sensor's static transform w.r.t. robot
            double sx, sy, sz, sroll, spitch, syaw;
            if( sensor_details.find("x") != sensor_details.end() )
                sx = sensor_details["x"];
            else
                sx = 0;
            if( sensor_details.find("y") != sensor_details.end() )
                sy = sensor_details["y"];
            else
                sy = 0;
            if( sensor_details.find("z") != sensor_details.end() )
                sz = sensor_details["z"];
            else
                sz = 0;
            if( sensor_details.find("roll") != sensor_details.end() )
                sroll = angles::from_degrees(sensor_details["roll"]);
            else
                sroll = 0;
            if( sensor_details.find("pitch") != sensor_details.end() )
                spitch = angles::from_degrees(sensor_details["pitch"]);
            else
                spitch = 0;
            if( sensor_details.find("yaw") != sensor_details.end() )
                syaw = angles::from_degrees(sensor_details["yaw"]);
            else
                syaw = 0;
            tf::Transform t_s_r = computeTransform(sx,sy,sz,sroll,spitch,syaw);

            SensorSpec sensor_spec(rmin, rmax, hfov, vfov);
            sensor_spec.ray_res = rres;
            Sensor::Ptr sensor(new Sensor(sensor_spec, t_s_r, sensor_name, robot_name));
            ros::Publisher pub = private_nh_->advertise<sensor_msgs::PointCloud2>(sensor->getSensorFQName(), 10);
            sensor_msg_pubs_.insert(std::make_pair(sensor->getSensorFQName(), pub));

            robot->addSensor(sensor);
        }

        // store the robot
        robots_.insert(std::make_pair(robot_name, robot));

        // for each robot create a navigator and store it
        Navigator::Ptr navigator(new Navigator(robot));

        std::string map_srv_name;
        private_nh_->param(robot_name + "/map_srv_name", map_srv_name, std::string("octomap_binary"));
        ROS_INFO_STREAM(robot_name << " : Waiting for service  " << map_srv_name << " to become available!");
        ros::service::waitForService(map_srv_name);        
        navigator->setMapServiceName(map_srv_name, *nh_);
        navigators_.insert(std::make_pair(robot_name, navigator));        

        // for each navigator, register a plan callback and store it
        ros::Subscriber plan_sub = private_nh_->subscribe<nav_msgs::Path>(robot_name + "/plan", 1, &Navigator::planCallback, navigator.get());
        nav_cmd_subs_.insert(std::make_pair(robot_name, plan_sub));
        // for each navigator, advertise a topic to publish its status
        ros::Publisher nav_status_pub = private_nh_->advertise<NavigationStatus>(robot_name + "/plan_status", 1);
        nav_status_pubs_.insert(std::make_pair(robot_name, nav_status_pub));
    }
}

void
Simulator::publish_tf()
{
    ros::Rate rate(50);  // publish at 50Hz
    while(ros::ok())
    {        
        // publish all robot pose transforms
        //for(auto robot: robots_)
        for(boost::unordered_map<std::string, Robot::Ptr>::iterator rit = robots_.begin(); rit != robots_.end(); rit++)
        {
            std::pair<std::string, Robot::Ptr> robot = *rit;
            tbr_.sendTransform(tf::StampedTransform(robot.second->getRobotPose(), ros::Time::now(), fixed_frame_, robot.second->getRobotFrameName()));

            std::vector<Sensor::Ptr> sensors = robot.second->getSensors();
            for(std::vector<Sensor::Ptr>::iterator sit = sensors.begin(); sit != sensors.end(); sit++)
            {
                Sensor::Ptr sensor = *sit;
                tbr_.sendTransform(tf::StampedTransform(sensor->getRelativeTransform(), ros::Time::now(), robot.second->getRobotFrameName(), sensor->getSensorName()));
            }
        }

        rate.sleep();
    }
}
