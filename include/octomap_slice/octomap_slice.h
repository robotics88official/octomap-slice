/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef OCTOMAP_SLICE_H_
#define OCTOMAP_SLICE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

namespace octomapSlice {
/**
 * @class OctomapSlice
 * @brief A class for processing octomaps into slices at fixed altitude
 */
class OctomapSlice {

    public:
        OctomapSlice(ros::NodeHandle& node);
        ~OctomapSlice();

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        float slice_height_;
        float slice_threshold_;
        bool slice_initialized_;
        nav_msgs::OccupancyGrid slice_map_;

        std::string octomap_topic_;
        ros::Publisher slice_publisher_;

        octomap::OcTreeKey padded_min_key_;
        unsigned tree_depth_;
        unsigned max_tree_depth_;
        octomap::OcTreeKey m_updateBBXMin;
        octomap::OcTreeKey m_updateBBXMax;

        // Projected map and octomap synced callback
        message_filters::Subscriber<nav_msgs::OccupancyGrid> grid_subscriber_;
        message_filters::Subscriber<octomap_msgs::Octomap> octo_subscriber_;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, octomap_msgs::Octomap> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        void syncedMapCallback(const nav_msgs::OccupancyGridConstPtr &grid_msg, const octomap_msgs::OctomapConstPtr &octo_msg);

        void initializeSlice();

    protected:
        inline unsigned mapIdx(int i, int j) const {
            return slice_map_.info.width * j + i;

        }
        inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
            return mapIdx((key[0] - padded_min_key_[0]),
                        (key[1] - padded_min_key_[1]));

        }
};

}

#endif