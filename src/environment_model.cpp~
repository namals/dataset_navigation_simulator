#include <dataset_navigation_simulator/environment_model.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace dataset_navigation_simulator;

EnvironmentModel::EnvironmentModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution)
    : cloud_(cloud)
    , cld_idx_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution))
    , point_resolution_(resolution)
    , octree_(NULL)
{
    cld_idx_->setInputCloud(cloud_);
    cld_idx_->addPointsFromInputCloud();
}

EnvironmentModel::EnvironmentModel(octomap::OcTree* octree)
    : cloud_(pcl::PointCloud<pcl::PointXYZ>::Ptr())
    , octree_(octree)
    , point_resolution_(octree->getResolution())
{    
}

void
EnvironmentModel::getSensorFrustum(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    if( cloud_ )
        getSensorFrustumFromPC(sensor_pose, sensor_spec, sensor_msg);
    else
        getSensorFrustumFromOctomap(sensor_pose, sensor_spec, sensor_msg);
}

void
EnvironmentModel::getSensorFrustumFromPC(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    tf::Vector3 sensor_translation = sensor_pose.getOrigin();
    Eigen::Vector3f origin(sensor_translation.x(),
                           sensor_translation.y(),
                           sensor_translation.z());

    // transform the sensor's edge points given in sensor specs to sensor's new frame
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensor_pose, sensorToWorld);
    pcl::PointCloud<pcl::PointXYZ> sensor_fov_edge_pts;
    pcl::transformPointCloud(*(sensor_spec.sensorFOVEdgePoints), sensor_fov_edge_pts, sensorToWorld);

    pcl::PointCloud<pcl::PointXYZ> frustum_pcd;
    frustum_pcd.is_dense = true;
    frustum_pcd.height = 1;
    
    for(pcl::PointXYZ& p: sensor_fov_edge_pts.points)
    {
        pcl::PointCloud<pcl::PointXYZ>::VectorType v;
        Eigen::Vector3f direction = p.getVector3fMap() - origin;
        direction.normalize();
        
        if( cld_idx_->getIntersectedVoxelCenters(origin, direction, v, 1 ) > 0 )
        {
            frustum_pcd.points.push_back(v[0]);
        }
    }        
    frustum_pcd.width = frustum_pcd.points.size();
    
    // TODO : transform frustum back to sensor's frame

    pcl::toROSMsg(frustum_pcd, sensor_msg);
}

void
EnvironmentModel::getSensorFrustumFromOctomap(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    tf::Vector3 sensor_translation = sensor_pose.getOrigin();
    octomap::point3d origin(sensor_translation.x(),
                            sensor_translation.y(),
                            sensor_translation.z());

    // transform the sensor's edge points given in sensor specs to sensor's new frame
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensor_pose, sensorToWorld);
    pcl::PointCloud<pcl::PointXYZ> sensor_fov_edge_pts;
    pcl::transformPointCloud(*(sensor_spec.sensorFOVEdgePoints), sensor_fov_edge_pts, sensorToWorld);

    octomap::point3d terminatePoint;
    pcl::PointCloud<pcl::PointXYZ> frustum_pcd;
    frustum_pcd.is_dense = true;
    frustum_pcd.height = 1;

    for(pcl::PointXYZ& p: sensor_fov_edge_pts.points)
    {
        octomap::point3d endPoint(p.x, p.y, p.z);
        octomap::point3d direction = endPoint - origin;
        direction.normalize();
        octomap::point3d startPoint = origin + direction*static_cast<float>(sensor_spec.r_min);
        if( octree_->castRay(startPoint, direction, terminatePoint, true, sensor_spec.r_max - sensor_spec.r_min) )
        {
            frustum_pcd.points.push_back(pcl::PointXYZ(terminatePoint.x(),
                                                       terminatePoint.y(),
                                                       terminatePoint.z()));
        }        
    }
    frustum_pcd.width = frustum_pcd.points.size();

    // TODO : transform frustum back to sensor's frame
    Eigen::Matrix4f worldToSensor;
    pcl_ros::transformAsMatrix(sensor_pose.inverse(), worldToSensor);
    pcl::PointCloud<pcl::PointXYZ> frustum_pcd_sensor;
    frustum_pcd_sensor.is_dense = true;
    frustum_pcd_sensor.height = 1;

    pcl::transformPointCloud(frustum_pcd, frustum_pcd_sensor, worldToSensor);
    frustum_pcd_sensor.width = frustum_pcd_sensor.points.size();

    pcl::toROSMsg(frustum_pcd_sensor, sensor_msg);
}


