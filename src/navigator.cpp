#include <dataset_navigation_simulator/navigator.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <angles/angles.h>

using namespace dataset_navigation_simulator;

void
Navigator::step()
{
    std::string rname = robot_->getRobotFrameName();
    ROS_INFO_STREAM(rname << " : navigator step");
    
    if( !robot_ )
        return;

    if( nav_status_.status == NavigationStatus::SUCCEEDED ||
        nav_status_.status == NavigationStatus::ABORTED )
        nav_status_.status = NavigationStatus::WAITING;

    // step if the current plan is still active
    if(nav_status_.status != NavigationStatus::ACTIVE)
    {
        ROS_INFO_STREAM(rname << " : plan is inactive");
        return;
    }

    ROS_INFO_STREAM(rname << " : plan is active");
    // get the new octomap data

    if( !map_service_client_.isValid() )
    {
        ROS_INFO_STREAM(rname << " : map service " << map_service_name_ << " is unavailable!");
        return;
    }
    
    octomap_msgs::GetOctomap srv;    
    if( !map_service_client_.call(srv) )
    {
        ROS_INFO_STREAM(rname << " : map info unavailable with service name " << map_service_name_);
        return;
    }

    ROS_INFO_STREAM(rname << " : map info available");
    octomap::OcTree* octree = static_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(srv.response.map));    

    double d_len_ = 0;
    double d_theta_ = 0;
    tf::Transform pose = curr_pose_;  // initialize the pose
    tf::Transform npose_tf;           // next pose along the path

    double rradius = robot_->getRobotRadius();
    int lookahead_idx = plan_idx_ + 1;
    bool in_collision = false;
    while( d_len_ < d_len_thresh_ && d_theta_ < d_theta_thresh_ && plan_.poses.size()-1 > plan_idx_ && plan_.poses.size()-1 > lookahead_idx )
    {
        // get the next pose in the planned path
        geometry_msgs::Pose npose = plan_.poses[lookahead_idx].pose;
        // check if this pose results in a collision       
        octomap::point3d bbmax(npose.position.x + rradius, npose.position.y + rradius, npose.position.z + rradius);
        octree->setBBXMax(bbmax);
        octomap::point3d bbmin(npose.position.x - rradius, npose.position.y - rradius, npose.position.z - rradius);
        octree->setBBXMin(bbmin);
        
        for(octomap::OcTree::leaf_bbx_iterator lit = octree->begin_leafs_bbx(bbmin, bbmax, octree->getTreeDepth()); lit != octree->end_leafs_bbx() && !in_collision; lit++ )
        {
            // check if any one these points is occupied and is within the radius
            octomap::OcTreeKey key = lit.getKey();
            if( octree->isNodeOccupied(*lit))
                in_collision = true;
        }

        if(in_collision)
            break;

        // else update the lookahead index for next iteration
        lookahead_idx++;
        
        // update d_len and d_theta_
        tf::poseMsgToTF(npose, npose_tf);
        d_len_ += (npose_tf.getOrigin() - pose.getOrigin()).length();
        d_theta_ += fabs( angles::shortest_angular_distance( tf::getYaw(npose_tf.getRotation()), tf::getYaw(pose.getRotation()) ) );
        // update pose
        pose = npose_tf;
    }

    // jump to next valid pose if not in collision
    if( !in_collision)
    {
        plan_idx_++; 
        tf::poseMsgToTF(plan_.poses[plan_idx_].pose, curr_pose_);
        robot_->setRobotPose(curr_pose_);
        if(plan_idx_ == plan_.poses.size()-1 )
            nav_status_.status = NavigationStatus::SUCCEEDED;

        ROS_INFO_STREAM(rname << " : robot successfully stepped along the planned path");
    }
    else
    {
        // path is in collision, abort the navigation
        ROS_INFO_STREAM(rname << " : robot is on a collision course, aborting plan");
        nav_status_.status = NavigationStatus::ABORTED;
    }
}

void
Navigator::setMapServiceName(std::string map_service_name, ros::NodeHandle& nh)
{
    map_service_name_ = map_service_name;

    // make a persistent service client
    map_service_client_ = nh.serviceClient<octomap_msgs::GetOctomap>(map_service_name, true);
}

void
Navigator::planCallback(const nav_msgs::Path::ConstPtr& path)
{
    ROS_INFO("Plan callback");
    plan_ = *path;
    plan_idx_ = 0;
    nav_status_.status = NavigationStatus::ACTIVE;
}
