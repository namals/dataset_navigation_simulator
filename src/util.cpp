#include <dataset_navigation_simulator/util.h>
#include <tf/transform_datatypes.h>

namespace dataset_navigation_simulator
{

    tf::Transform computeTransform(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf::Transform t;
        t.setOrigin(tf::Vector3(x,y,z));
        t.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));
        return t;
    }

    void ExtractAllBasicVoxels(octomap::OcTreeKey key,
                               uint32_t depth,
                               uint32_t max_tree_depth,
                               octomap::KeySet& key_set)
    {
        // get the leaf voxel's (cube's) size in smallest voxels (i.e. resolution)
        int cube_size_in_voxels = (max_tree_depth - depth) << 1;

        int s = cube_size_in_voxels/2;

        // for voxels with depth < max_tree_depth, their key corresponds is 
        // generated from bottom south west coordinate of the top north east octant
    
        for(int i = -s; i < s; i++)
        {
            for(int j = -s; j < s; j++)
            {
                for(int l = -s; l < s; l++)
                {
                    octomap::OcTreeKey nkey;
                    nkey.k[0] = key.k[0] + i;
                    nkey.k[1] = key.k[1] + j;
                    nkey.k[2] = key.k[2] + l;               
                    key_set.insert(nkey);
                }
            }
        }
    }


    void octomapToPCLPointCloud(octomap::OcTree* octree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd)
    {
        uint32_t tree_depth = octree->getTreeDepth();
        octomap::KeySet surface_voxels;

        for(octomap::OcTree::iterator it = octree->begin(tree_depth),
                end = octree->end(); it != end; it++)
        {
            if( !octree->isNodeOccupied(*it) )
                continue;

            octomap::KeySet key_set;
            uint32_t depth = it.getDepth();
            octomap::OcTreeKey key = it.getKey();

            if( depth < tree_depth )
                ExtractAllBasicVoxels(key, depth, tree_depth, key_set);
            else
                key_set.insert(key);

            surface_voxels.insert( key_set.begin(), key_set.end() );
        }

        pcd->is_dense = true;
        pcd->height = 1;
        pcd->points.resize(surface_voxels.size());
        int i = 0;
        //for(octomap::OcTreeKey key: surface_voxels)
        for(octomap::KeySet::iterator kit = surface_voxels.begin(); kit != surface_voxels.end(); kit++ )
        {
            octomap::OcTreeKey key = *kit;
            octomap::point3d op = octree->keyToCoord(key);
            pcl::PointXYZ p(op.x(), op.y(), op.z());
            pcd->points[i++] = p;
        }
        pcd->width = pcd->points.size();
    }
    
}
