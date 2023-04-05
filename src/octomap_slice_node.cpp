/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "octomap_slice/octomap_slice.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_slice");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  octomapSlice::OctomapSlice octomap_slice(node);

  ros::spin();

  return 0;
}