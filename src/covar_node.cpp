#include <ros/ros.h>
#include "covar_ros.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "covar");
    ros::NodeHandle nh("~");

    covar_ros covar_ros_obj;

    // declarations
    std::string input_odom;
    std::string output_odom;
    std::string pose_covar;
    std::string twist_covar;
    
    // get parameters
    nh.param<std::string>("input_odom", input_odom, "/integrated_to_init");
    nh.param<std::string>("output_odom", output_odom, "/loam_odom_with_covar");
    nh.param<std::string>("pose_covar", pose_covar, "");
    nh.param<std::string>("twist_covar", twist_covar, "");

    if (covar_ros_obj.setup(nh, input_odom, output_odom, pose_covar, twist_covar)) {
        // initialization successful
        ros::spin();
    }

    return 0;
}
