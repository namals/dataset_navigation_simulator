#include <dataset_navigation_simulator/environment_model.h>
#include <angles/angles.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/range_image/range_image.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <vector>

using namespace dataset_navigation_simulator;

EnvironmentModel::EnvironmentModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution)
    : cloud_(cloud)    
    , cloud_index_(new pcl::search::KdTree<pcl::PointXYZ>())
    , point_resolution_(resolution)
    , octree_(NULL)
    , frustum_culling_(new pcl::FrustumCulling<pcl::PointXYZ>())
{
    frustum_culling_->setInputCloud(cloud_);
    cloud_index_->setInputCloud(cloud_);
}

EnvironmentModel::EnvironmentModel(octomap::OcTree* octree)
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>())
    , octree_(octree)
    , point_resolution_(octree->getResolution())
    , frustum_culling_(new pcl::FrustumCulling<pcl::PointXYZ>())
{
    // create a point cloud from octomap    
    octomapToPCLPointCloud(octree_, cloud_);
    frustum_culling_->setInputCloud(cloud_);
}

void
EnvironmentModel::getSensorFrustum(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    // set sensor's specs to culler
    frustum_culling_->setVerticalFOV(angles::to_degrees(sensor_spec.ver_fov));
    frustum_culling_->setHorizontalFOV(angles::to_degrees(sensor_spec.hor_fov));
    frustum_culling_->setNearPlaneDistance(sensor_spec.r_min);
    frustum_culling_->setFarPlaneDistance(sensor_spec.r_max);

    // compute the sensor's pose as a Matrix4
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensor_pose, sensorToWorld);
    // set sensor's pose to culler
    frustum_culling_->setCameraPose(sensorToWorld);

    pcl::PointCloud<pcl::PointXYZ> frustum_pcd;
    frustum_culling_->filter(frustum_pcd);

    pcl::PointCloud<pcl::PointXYZ> frustum_pcd_visible;
    frustum_pcd_visible.is_dense = true;
    frustum_pcd_visible.height = 1;
    frustum_pcd_visible.points.resize(frustum_pcd.points.size());

    octomap::point3d origin(sensor_pose.getOrigin().x(), sensor_pose.getOrigin().y(), sensor_pose.getOrigin().z());
    double octree_res = octree_->getResolution();
    double tolerance = 4*octree_res*sqrt(3.0);
    size_t k = 0;
    for(size_t i = 0; i < frustum_pcd.width; i++)
    {
        pcl::PointXYZ p = frustum_pcd[i];
        octomap::point3d op(p.x, p.y, p.z);
        octomap::point3d dir = op - origin;
        octomap::point3d hitp;
        if( octree_->castRay(origin, dir, hitp, true, sensor_spec.r_max + 1.0) )
        {
            if( op.distance(hitp) <= octree_res  )
                frustum_pcd_visible[k++] = p;
            else if( (hitp-op).dot(dir) < 0 && op.distance(hitp) <= tolerance )
                frustum_pcd_visible[k++] = p;
        }
    }
    frustum_pcd_visible.points.resize(k);
    frustum_pcd_visible.width = k;

//    // TODO : Remove points that are occluded
//    // Use pcl::RangeImage to get the unoccluded point cloud
//    float max_angle_width = (float)(360.0 * (M_PI/180.0f));
//    float max_angle_height = (float)(180.0 * (M_PI/180.0f));
//    float angular_res = (float)(0.1 * (M_PI/180.0f));
//    float angular_res_x = (float)(sensor_spec.hor_fov/640.0f);
//    float angular_res_y = (float)(sensor_spec.ver_fov/480.0f);
//    Eigen::Matrix3f R;
//    R = Eigen::AngleAxisf(90 * M_PI/180.0f, Eigen::Vector3f::UnitX()) *
//            Eigen::AngleAxisf(90 * M_PI/180.0f, Eigen::Vector3f::UnitY()) *
//            Eigen::AngleAxisf(0 * M_PI/180.0f, Eigen::Vector3f::UnitZ());

//    pcl::RangeImage visible;
//    //Eigen::Affine3f sp = (Eigen::Affine3f)sensorToWorld;

//    //visible.createFromPointCloud(frustum_pcd, angular_res_x, angular_res_y, max_angle_width, max_angle_height, sp, pcl::RangeImage::LASER_FRAME);


    Eigen::Matrix4f worldToSensor;
    pcl_ros::transformAsMatrix(sensor_pose.inverse(), worldToSensor);
    pcl::PointCloud<pcl::PointXYZ> frustum_pcd_sensor;
    frustum_pcd_sensor.is_dense = true;
    frustum_pcd_sensor.height = 1;

    pcl::transformPointCloud(frustum_pcd_visible, frustum_pcd_sensor, worldToSensor);
    frustum_pcd_sensor.width = frustum_pcd_sensor.points.size();

//    visible.createFromPointCloud(frustum_pcd_sensor, angular_res, max_angle_width, max_angle_height, Eigen::Affine3f::Identity (), pcl::RangeImage::LASER_FRAME);
//    //visible.createFromPointCloud(frustum_pcd_sensor, angular_res_x, angular_res_y, max_angle_width, max_angle_height, Eigen::Affine3f::Identity (), pcl::RangeImage::LASER_FRAME);
//    std::cout << "visible size : " << visible.size() << std::endl;
//    std::cout << "image width : " << visible.width << std::endl;
//    std::cout << "image height : " << visible.height << std::endl;



////    pcl::PointCloud<pcl::PointXYZ> frustum_visible_pcd_sensor;
////    frustum_visible_pcd_sensor.is_dense = false;
////    frustum_visible_pcd_sensor.width = visible.width;
////    frustum_visible_pcd_sensor.height = visible.height;

////    pcl::transformPointCloud(visible, frustum_visible_pcd_sensor, worldToSensor);
//    pcl::toROSMsg(visible, sensor_msg);





    pcl::toROSMsg(frustum_pcd_sensor, sensor_msg);
    
    // if( cloud_ )
    //     getSensorFrustumFromPC(sensor_pose, sensor_spec, sensor_msg);
    // else
    //     getSensorFrustumFromOctomap(sensor_pose, sensor_spec, sensor_msg);
}

void
EnvironmentModel::getSensorFrustumFromPC(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    // tf::Vector3 sensor_translation = sensor_pose.getOrigin();
    // Eigen::Vector3f origin(sensor_translation.x(),
    //                        sensor_translation.y(),
    //                        sensor_translation.z());

    // // transform the sensor's edge points given in sensor specs to sensor's new frame
    // Eigen::Matrix4f sensorToWorld;
    // pcl_ros::transformAsMatrix(sensor_pose, sensorToWorld);
    // pcl::PointCloud<pcl::PointXYZ> sensor_fov_edge_pts;
    // pcl::transformPointCloud(*(sensor_spec.sensorFOVEdgePoints), sensor_fov_edge_pts, sensorToWorld);

    // pcl::PointCloud<pcl::PointXYZ> frustum_pcd;
    // frustum_pcd.is_dense = true;
    // frustum_pcd.height = 1;
    
    // for(pcl::PointXYZ& p: sensor_fov_edge_pts.points)
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::VectorType v;
    //     Eigen::Vector3f direction = p.getVector3fMap() - origin;
    //     direction.normalize();
        
    //     if( cld_idx_->getIntersectedVoxelCenters(origin, direction, v, 1 ) > 0 )
    //     {
    //         frustum_pcd.points.push_back(v[0]);
    //     }
    // }        
    // frustum_pcd.width = frustum_pcd.points.size();
    
    // // TODO : transform frustum back to sensor's frame

    // pcl::toROSMsg(frustum_pcd, sensor_msg);
}

void
EnvironmentModel::getSensorFrustumFromOctomap(const tf::Transform& sensor_pose, const SensorSpec& sensor_spec, sensor_msgs::PointCloud2& sensor_msg)
{
    // tf::Vector3 sensor_translation = sensor_pose.getOrigin();
    // octomap::point3d origin(sensor_translation.x(),
    //                         sensor_translation.y(),
    //                         sensor_translation.z());

    // // transform the sensor's edge points given in sensor specs to sensor's new frame
    // Eigen::Matrix4f sensorToWorld;
    // pcl_ros::transformAsMatrix(sensor_pose, sensorToWorld);
    // pcl::PointCloud<pcl::PointXYZ> sensor_fov_edge_pts;
    // pcl::transformPointCloud(*(sensor_spec.sensorFOVEdgePoints), sensor_fov_edge_pts, sensorToWorld);

    // octomap::point3d terminatePoint;
    // pcl::PointCloud<pcl::PointXYZ> frustum_pcd;
    // frustum_pcd.is_dense = true;
    // frustum_pcd.height = 1;

    // for(pcl::PointXYZ& p: sensor_fov_edge_pts.points)
    // {
    //     octomap::point3d endPoint(p.x, p.y, p.z);
    //     octomap::point3d direction = endPoint - origin;
    //     direction.normalize();
    //     octomap::point3d startPoint = origin + direction*static_cast<float>(sensor_spec.r_min);
    //     if( octree_->castRay(startPoint, direction, terminatePoint, true, sensor_spec.r_max - sensor_spec.r_min) )
    //     {
    //         frustum_pcd.points.push_back(pcl::PointXYZ(terminatePoint.x(),
    //                                                    terminatePoint.y(),
    //                                                    terminatePoint.z()));
    //     }        
    // }
    // frustum_pcd.width = frustum_pcd.points.size();

    // // TODO : transform frustum back to sensor's frame
    // Eigen::Matrix4f worldToSensor;
    // pcl_ros::transformAsMatrix(sensor_pose.inverse(), worldToSensor);
    // pcl::PointCloud<pcl::PointXYZ> frustum_pcd_sensor;
    // frustum_pcd_sensor.is_dense = true;
    // frustum_pcd_sensor.height = 1;

    // pcl::transformPointCloud(frustum_pcd, frustum_pcd_sensor, worldToSensor);
    // frustum_pcd_sensor.width = frustum_pcd_sensor.points.size();

    // pcl::toROSMsg(frustum_pcd_sensor, sensor_msg);
}



bool
EnvironmentModel::isRobotInCollision(const tf::Transform &robot_pose, float robot_radius)
{
    tf::Vector3 rpos = robot_pose.getOrigin();
    pcl::PointXYZ q;
    q.x = rpos.x();
    q.y = rpos.y();
    q.z = rpos.z();

    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_distances;
    return cloud_index_->radiusSearch(q, robot_radius, nn_indices, nn_sqr_distances) > 0;
}
