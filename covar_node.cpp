#include <ros/ros.h>
#include "covar/covar_ros.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "covar");
  ros::NodeHandle nh;

  covar_ros covar_ros_obj;

  if (covar_ros_obj.setup(nh)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
