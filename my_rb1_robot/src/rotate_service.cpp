// includes

#include "geometry_msgs/Twist.h"
#include "my_rb1_msgs/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/tf.h"

double current_yaw = 0.0;
bool odom_got = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double q_x = msg->pose.pose.orientation.x;
  double q_y = msg->pose.pose.orientation.y;
  double q_z = msg->pose.pose.orientation.z;
  double q_w = msg->pose.pose.orientation.w;

  // convert quaternion to Euler Angles
  tf::Quaternion quat(q_x, q_y, q_z, q_w);
  double roll, pitch;
  tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);
  odom_got = true;
}

bool rotate_callback(my_rb1_msgs::Rotate::Request &req,
                     my_rb1_msgs::Rotate::Response &res) {

  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Rate rate(10);

  if (!odom_got) {
    res.result = "No Odom data received.";
    return false;
  }

  // convert degrees into radians

  double target_angle = req.degrees * M_PI / 180;

  double initial_yaw = current_yaw;

  double target_yaw = initial_yaw + target_angle;

  // Rotate the robot

  geometry_msgs::Twist vel_msg;
  vel_msg.angular.z = (req.degrees > 0) ? 0.5 : -0.5;
  while (ros::ok()) {
    double error = target_yaw - current_yaw;

    if (fabs(error) < 0.05) {
      vel_msg.angular.z = 0.0;
      cmd_vel_pub.publish(vel_msg);
      break;
    }
    cmd_vel_pub.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }

  res.result = "Rotation complete!";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_server");
  ros::NodeHandle nh;

  // subscribe to odom topic
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

  // Create the service server

  ros::ServiceServer service =
      nh.advertiseService("/rotate_robot", rotate_callback);
  ROS_INFO("Ready to rotate the robot.");

  ros::spin();

  return 0;
}
