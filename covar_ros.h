#ifndef COVAR_ROS_H_
#define COVAR_ROS_H_

#include "covar.h"
#include <ros/ros.h>

/* Continuing from covar.h, what we want to do here is
 * 1) Instantiate a covar object for each part of the odometry topic
 *    (http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
 *    a) Pose
 *    b) Twist
 * 2) Service a ROS service request for
 *    a) Setting a Pose Covariance 
 *    b) Setting a Twist Covariance
 * 3) Indicate whether the covariance is set, which means we need flags
 *    a) Pose
 *    b) Twist
 * 4) Subscribe to an odometry topic
 * 5) Publish an odometry topic, appending the covariance matrix
 */

class covar_ros
{
    private:
        covar* pose_covar;
        covar* twist_covar;
        bool pose_covar_available;
        bool twist_covar_available;
        ros::subscriber odom_sub;
        ros::publisher odom_pub;
        ros::ServiceServer generate_pose_covar;
        ros::ServiceServer generate_twist_covar;
        ros::ServiceServer generate_both_covar;
    public:
        

};

#endif // COVAR_ROS_H_