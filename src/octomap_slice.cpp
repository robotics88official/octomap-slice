/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "octomap_slice/octomap_slice.h"

#include <octomap_msgs/conversions.h>
#include <nav_msgs/MapMetaData.h>

namespace octomapSlice
{
OctomapSlice::OctomapSlice(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , slice_height_(2.0)
  , slice_threshold_(0.5)
  , slice_initialized_(false)
  , octomap_topic_("/octomap_binary")
  , tree_depth_(0)
  , max_tree_depth_(0)
{
    // Set params from launch file 
    private_nh_.param<std::string>("octomap_topic", octomap_topic_, octomap_topic_);
    private_nh_.param<float>("slice_height", slice_height_, slice_height_);

    slice_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("octomap_slice", 10); // TODO expose param

    grid_subscriber_.subscribe(nh_, "projected_map", 10);
    octo_subscriber_.subscribe(nh_, octomap_topic_, 10);
    sync_.reset(new Sync(MySyncPolicy(10), grid_subscriber_, octo_subscriber_));
    sync_->registerCallback(boost::bind(&OctomapSlice::syncedMapCallback, this, _1, _2));
}

OctomapSlice::~OctomapSlice(){}

void OctomapSlice::syncedMapCallback(const nav_msgs::OccupancyGridConstPtr &grid_msg, const octomap_msgs::OctomapConstPtr &octo_msg) {
    nav_msgs::OccupancyGrid floor_grid = *grid_msg;
    slice_map_.header = floor_grid.header;
    slice_map_.info = floor_grid.info;
    slice_map_.data.clear();
    // init to unknown:
    slice_map_.data.resize(slice_map_.info.width * slice_map_.info.height, -1);

    octomap::OcTree* octree = (octomap::OcTree*) octomap_msgs::msgToMap(*octo_msg);
    
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricMax(maxX, maxY, maxZ);
    tree_depth_ = octree->getTreeDepth();
    max_tree_depth_ = tree_depth_;

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = octree->coordToKey(minPt, max_tree_depth_);
    octomap::OcTreeKey maxKey = octree->coordToKey(maxPt, max_tree_depth_);

    octomap::OcTreeKey padded_max_key;
    if (!octree->coordToKeyChecked(minPt, max_tree_depth_, padded_min_key_)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!octree->coordToKeyChecked(maxPt, max_tree_depth_, padded_max_key)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    if (octree)
    {
         for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
            if (octree->isNodeOccupied(*it)){
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > (slice_height_ - slice_threshold_) && z - half_size < (slice_height_ + slice_threshold_))
                {
                    bool occupied = octree->isNodeOccupied(*it);
                    if (it.getDepth() == max_tree_depth_){
                        unsigned idx = mapIdx(it.getKey());
                        if (occupied){
                            slice_map_.data[mapIdx(it.getKey())] = 100;
                        }
                        else if (slice_map_.data[idx] == -1){
                            slice_map_.data[idx] = 0;
                        }

                    } else{
                        int intSize = 1 << (max_tree_depth_ - it.getDepth());
                        octomap::OcTreeKey minKey=it.getIndexKey();
                        for(int dx=0; dx < intSize; dx++){
                        int i = (minKey[0]+dx - padded_min_key_[0]);
                        for(int dy=0; dy < intSize; dy++){
                            unsigned idx = mapIdx(i, (minKey[1]+dy - padded_min_key_[1]));
                            if (occupied){
                                slice_map_.data[idx] = 100;
                            }
                            else if (slice_map_.data[idx] == -1){
                                slice_map_.data[idx] = 0;
                            }
                        }
                        }
                    }
                }
            }
            else{ // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > (slice_height_ - slice_threshold_) && z - half_size < (slice_height_ + slice_threshold_))
                {
                    bool occupied = octree->isNodeOccupied(*it);
                    if (it.getDepth() == max_tree_depth_){
                        unsigned idx = mapIdx(it.getKey());
                        if (occupied){
                            slice_map_.data[mapIdx(it.getKey())] = 100;
                        }
                        else if (slice_map_.data[idx] == -1){
                            slice_map_.data[idx] = 0;
                        }

                    } else{
                        int intSize = 1 << (max_tree_depth_ - it.getDepth());
                        octomap::OcTreeKey minKey=it.getIndexKey();
                        for(int dx=0; dx < intSize; dx++){
                        int i = (minKey[0]+dx - padded_min_key_[0]);
                        for(int dy=0; dy < intSize; dy++){
                            unsigned idx = mapIdx(i, (minKey[1]+dy - padded_min_key_[1]));
                            if (occupied){
                                slice_map_.data[idx] = 100;
                            }
                            else if (slice_map_.data[idx] == -1){
                                slice_map_.data[idx] = 0;
                            }
                        }
                        }
                    }
                }
            }
        }
    }
    slice_publisher_.publish(slice_map_);
}

}