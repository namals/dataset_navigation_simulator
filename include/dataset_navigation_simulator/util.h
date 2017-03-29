#ifndef _UTIL_H_
#define _UTIL_H_

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/LinearMath/Transform.h>

namespace dataset_navigation_simulator
{

    struct SensorSpec
    {        
        double r_min;
        double r_max;
        double hor_fov;
        double ver_fov;
        double ray_res;

        pcl::PointCloud<pcl::PointXYZ>::Ptr sensorFOVEdgePoints;

        SensorSpec()
        : r_min(0.5) , r_max(5.0), hor_fov(1.01), ver_fov(0.79), ray_res(0.028571429), sensorFOVEdgePoints(new pcl::PointCloud<pcl::PointXYZ>)
        {
            generateSensorFOVEdgePoints();
        }

    SensorSpec(double rmin = 0.5, double rmax = 5.0, double hfov = 1.01, double vfov = 0.79)
        : r_min(rmin), r_max(rmax), hor_fov(hfov), ver_fov(vfov), ray_res(0.028571429), sensorFOVEdgePoints(new pcl::PointCloud<pcl::PointXYZ>)
        {
            generateSensorFOVEdgePoints();
        }

        SensorSpec(const SensorSpec& spec)
        : r_min(spec.r_min), r_max(spec.r_max), hor_fov(spec.hor_fov), ver_fov(spec.ver_fov), ray_res(spec.ray_res), sensorFOVEdgePoints(new pcl::PointCloud<pcl::PointXYZ>(*(spec.sensorFOVEdgePoints)))
        {
        }

    private:
        void generateSensorFOVEdgePoints()
        {
            sensorFOVEdgePoints->is_dense = true;
            sensorFOVEdgePoints->height = 1;

            int hfov_steps_2 = static_cast<int>(floor(0.5*hor_fov/ray_res));
            int vfov_steps_2 = static_cast<int>(floor(0.5*ver_fov/ray_res));
            double alpha, beta;
            int v_step_lb = -vfov_steps_2;
            double range = r_max;
            
            pcl::PointXYZ p;
            for (int i = v_step_lb; i <= vfov_steps_2; i++)
            {
                beta = ray_res*i;
                for (int j = -hfov_steps_2; j <= hfov_steps_2; j++){
                    alpha = ray_res*j;
                    p.x = range*cos(beta)*cos(alpha);
                    p.y = range*cos(beta)*sin(alpha);
                    p.z = range*sin(beta);
                    
                    sensorFOVEdgePoints->points.push_back(p);
                }
            }
            sensorFOVEdgePoints->width = sensorFOVEdgePoints->points.size();
        }
    };

    tf::Transform computeTransform(double x, double y, double z, double roll, double pitch, double yaw);
}

#endif
