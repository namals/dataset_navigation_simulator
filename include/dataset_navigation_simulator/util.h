#ifndef _DATASET_NAVIGATOR_SIMULATOR_UTIL_H_
#define _DATASET_NAVIGATOR_SIMULATOR_UTIL_H_

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>
#include <octomap/octomap.h>

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

        void getSensorFOVCorners(std::vector<geometry_msgs::Point>& fov_corners)
                {
                    // use this only if the sensor is an RGB-D sensor
                    fov_corners.resize(8);

                    double hor_fov_2 = hor_fov/2;
                    double ver_fov_2 = ver_fov/2;

                    // near plane values
                    geometry_msgs::Point np;
                    np.x = r_min;
                    np.y = np.x*tan(hor_fov_2);
                    np.z = np.x*tan(ver_fov_2);

                    // far plane values
                    geometry_msgs::Point fp;
                    fp.x = r_max;
                    fp.y = fp.x*tan(hor_fov_2);
                    fp.z = fp.x*tan(ver_fov_2);

                    geometry_msgs::Point p;
                    p.x = np.x;
                    p.z = -np.z;

                    // near plane bottom left
                    p.y = np.y;
                    fov_corners[0] = p;
                    // near plane bottom right
                    p.y = -np.y;
                    fov_corners[1] = p;

                    p.z = np.z;
                    // near plane top right
                    fov_corners[2] = p;
                    // near plane top left
                    p.y = np.y;
                    fov_corners[3] = p;

                    p.x = fp.x;
                    p.z = -fp.z;

                    // far plane bottom left
                    p.y = fp.y;
                    fov_corners[4] = p;
                    // far plane bottom right
                    p.y = -fp.y;
                    fov_corners[5] = p;

                    p.z = fp.z;
                    // far plane top right
                    fov_corners[6] = p;
                    // far plane top left
                    p.y = fp.y;
                    fov_corners[7] = p;
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

    /*!
     * \brief Convert obstacle points in the octomap to a PCL point cloud
     */
    void octomapToPCLPointCloud(octomap::OcTree* octree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);

    void ExtractAllBasicVoxels(octomap::OcTreeKey key,
                               uint32_t depth,
                               uint32_t max_tree_depth,
                               octomap::KeySet& key_set);
}

#endif
