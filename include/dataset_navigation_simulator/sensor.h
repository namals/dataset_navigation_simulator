#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <tf/LinearMath/Transform.h>
#include <dataset_navigation_simulator/util.h>
#include <dataset_navigation_simulator/environment_model.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

namespace dataset_navigation_simulator
{
    class Sensor
    {
    public:
        typedef boost::shared_ptr<Sensor> Ptr;
        typedef boost::shared_ptr<const Sensor> ConstPtr;

        Sensor(const SensorSpec& sensor_spec, const tf::Transform& t_s_r, std::string name, std::string robot_name);

        std::string getSensorName();

        std::string getSensorFQName();

        void setSensorSpec(const SensorSpec& sensor_spec);

        SensorSpec getSensorSpec();

        void updateTransform(const tf::Transform& t_r_m);

        tf::Transform& getTransform();

        tf::Transform& getRelativeTransform();

        void spinOnce(EnvironmentModel::Ptr env);

        sensor_msgs::PointCloud2& getCurrentSensorData();

    protected:
        std::string name_;                    /*!< sensor's name */

        std::string fq_name_;                 /*!< sensor's fully qualified name including the robot it is bound to */

        SensorSpec sensor_spec_;

        tf::Transform t_s_r_;                  /*!< static transform of sensor w.r.t. robot */

        tf::Transform t_;                      /*!< sensor's transform w.r.t. world */

        sensor_msgs::PointCloud2 sensor_data_;

        boost::mutex mutex_;               
    };

} //~namespace

#endif
