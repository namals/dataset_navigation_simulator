#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/octree/octree_search.h>

#include <octomap/octomap.h>
#include <tf/transform_datatypes.h>

#include <dataset_navigation_simulator/util.h>

#include <memory>

namespace dataset_navigation_simulator
{
    
    class EnvironmentModel
    {
    public:
        typedef std::shared_ptr<EnvironmentModel> Ptr;
        typedef std::shared_ptr<const EnvironmentModel> ConstPtr;
        
        /*!
         * \brief Generate environment model using a point cloud
         */
        EnvironmentModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double res);

        /*!
         * \brief Generate environment model using an octomap
         */
        EnvironmentModel(octomap::OcTree* octree_);

        /*!
         * \brief Get the sensor frustum 
         */
        void getSensorFrustum(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg);
        
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;                   /*!< Point cloud storing the entire environment */
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr cld_idx_;  /*!< Point cloud's index */        

        octomap::OcTree* octree_;                                     /*!< Octomap representation of the environment */

        double point_resolution_;

        void getSensorFrustumFromPC(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg);

        void getSensorFrustumFromOctomap(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg);
    };
    
} //~namespace

#endif