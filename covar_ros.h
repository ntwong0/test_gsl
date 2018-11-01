#ifndef COVAR_ROS_H_
#define COVAR_ROS_H_

#include "covar.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

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
        covar pose_covar;
        covar twist_covar;
        double pose_covar_mat[36];
        double twist_covar_mat[36];

        uint8 pose_data_points_count;
        uint8 twist_data_points_count;
        uint8 pose_data_points_limit;
        uint8 twist_data_points_limit;
        bool pose_covar_available;
        bool twist_covar_available;

        ros::subscriber sub_odom;
        ros::publisher pub_odom;
        ros::ServiceServer gen_pose_covar;
        ros::ServiceServer gen_twist_covar;
        ros::ServiceServer gen_both_covar;
    
    public:
        covar_ros()
        {
            pose_data_points_count  = 0;
            twist_data_points_count = 0;
            pose_data_points_limit  = 0;
            twist_data_points_limit = 0;
            pose_covar_available    = false;
            twist_covar_available   = false;
        }

        void input_odom_handler(
                        const nav_msgs::Odometry::ConstPtr& input_odom_msg)
        {
            if(pose_data_points_count++ < pose_data_points_limit)
            {
                double roll, pitch, yaw;
                geometry_msgs::Quaternion tempQuat = 
                    input_odom_msg->pose.pose.orientation;
                tf::Matrix3x3(tf::Quaternion(
                    tempQuat.z, 
                    -tempQuat.x, 
                    -tempQuat.y, 
                    tempQuat.w)).getRPY(roll, pitch, yaw);

                double * pose_data_point = new double[6];
                pose_data_point[0] = input_odom_msg->pose.pose.position.x;
                pose_data_point[1] = input_odom_msg->pose.pose.position.y;
                pose_data_point[2] = input_odom_msg->pose.pose.position.z;
                pose_data_point[3] = roll;
                pose_data_point[4] = pitch;
                pose_data_point[5] = yaw;
                
                pose_covar.insert(pose_data_point);
            }

            if(pose_data_points_count > pose_data_points_limit 
                && pose_data_points_limit > 0)
            {
                pose_data_points_count = 0;
                pose_data_points_limit = 0;
                pose_covar.generate_covar();
                pose_covar.get_mat(pose_covar_mat);
                pose_covar_available = true;
            }

            if(twist_data_points_count++ < twist_data_points_limit)
            {
                double * twist_data_point = new double[6];
                twist_data_point[0] = 
                                twist_data_point->twist.twist.linear.x;
                twist_data_point[1] = 
                                twist_data_point->twist.twist.linear.y;
                twist_data_point[2] = 
                                twist_data_point->twist.twist.linear.z;
                twist_data_point[3] = 
                                twist_data_point->twist.twist.angular.x;
                twist_data_point[4] = 
                                twist_data_point->twist.twist.angular.y;
                twist_data_point[5] = 
                                twist_data_point->twist.twist.angular.z;
                
                twist_covar.insert(twist_data_point);
            }

            if(twist_data_points_count > twist_data_points_limit 
                && twist_data_points_limit > 0)
            {
                twist_data_points_count = 0;
                twist_data_points_limit = 0;
                twist_covar.generate_covar();
                twist_covar.get_mat(twist_covar_mat);
                twist_covar_available = true;
            }
        }

        // Pulled from ROS Service Tutorial
        //   http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
        // bool add(beginner_tutorials::AddTwoInts::Request  &req,
        //          beginner_tutorials::AddTwoInts::Response &res)
        // {
        //   res.sum = req.a + req.b;
        //   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
        //   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
        //   return true;
        // }

        bool setup(ros::NodeHandle& nh, std:string& input_odom, 
                    std:string& output_odom)
        {
            sub_odom = nh.subscribe<nav_msgs::Odometry>(input_odom, 5, 
                            &covar_ros::input_odom_handler, 
                            this);
            
            pub_odom = nh.advertise<nav_msgs::Odometry>(output_odom, 5);

            gen_pose_covar  = nh.advertiseService("gen_pose_covar", add);
            gen_twist_covar = nh.advertiseService("gen_twist_covar", add);
            gen_both_covar  = nh.advertiseService("gen_both_covar", add);

            return true;
        }

};

#endif // COVAR_ROS_H_
