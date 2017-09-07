#include <dataset_navigation_simulator/sensor.h>

using namespace dataset_navigation_simulator;

Sensor::Sensor(const SensorSpec& sensor_spec, const tf::Transform& t_s_r, std::string name, std::string robot_name)
    : sensor_spec_(sensor_spec)
    , t_s_r_(t_s_r)
    , name_(name)
    , fq_name_(robot_name + "/" + name)
{
}

std::string
Sensor::getSensorName()
{
    return name_;
}

std::string
Sensor::getSensorFQName()
{
    return fq_name_;
}

void
Sensor::setSensorSpec(const SensorSpec& sensor_spec)
{
    boost::unique_lock<boost::mutex> lock(mutex_);
    sensor_spec_ = sensor_spec;
}

SensorSpec
Sensor::getSensorSpec()
{
    boost::unique_lock<boost::mutex> lock(mutex_);
    return sensor_spec_;
}

void
Sensor::updateTransform(const tf::Transform& t_r_m)
{
    boost::unique_lock<boost::mutex> lock(mutex_);
    t_ = t_r_m*t_s_r_;
}

tf::Transform&
Sensor::getTransform()
{
    boost::unique_lock<boost::mutex> lock(mutex_);
    return t_;
}

tf::Transform&
Sensor::getRelativeTransform()
{
    boost::unique_lock<boost::mutex> lock(mutex_);
    return t_s_r_;
}


void
Sensor::spinOnce(EnvironmentModel::Ptr env)
{
    // get the sensor data from environment model using sensor_pose
    boost::unique_lock<boost::mutex> lock(mutex_);
    sensor_data_.data.clear();
    env->getSensorFrustum(t_, sensor_spec_, sensor_data_);
    sensor_data_.header.stamp = ros::Time::now();
    sensor_data_.header.frame_id = name_;
}

sensor_msgs::PointCloud2&
Sensor::getCurrentSensorData()
{
    return sensor_data_;
}
