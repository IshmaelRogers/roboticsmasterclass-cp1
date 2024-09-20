// File: rotate_server.cpp

// Includes
#include "geometry_msgs/Twist.h"
#include "my_rb1_msgs/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <iostream>
#include <mutex>

using namespace std;

class RotateServer {
public:
  RotateServer() : odom_got(false), rotating(false) {
    // Initialize publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to odometry
    odom_sub = nh.subscribe("/odom", 1000, &RotateServer::odomCallback, this);

    // Advertise service
    service = nh.advertiseService("/rotate_robot",
                                  &RotateServer::rotateCallback, this);

    ROS_INFO("Service Ready.");
  }

private:
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber odom_sub;
  ros::ServiceServer service;

  double current_yaw;
  bool odom_got;
  bool rotating;
  std::mutex mtx;

  // Callback to process odometry data
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx);
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;

    // Convert quaternion to Euler angles
    tf::Quaternion quat(q_x, q_y, q_z, q_w);
    double roll, pitch;
    tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);

    // Normalize yaw to [-pi, pi]
    current_yaw = normalizeAngle(current_yaw);
    odom_got = true;
  }

  // Service callback to handle rotation requests
  bool rotateCallback(my_rb1_msgs::Rotate::Request &req,
                      my_rb1_msgs::Rotate::Response &res) {
    // Lock mutex to check and set rotation state
    {
      //cout << "Degrees: " << req.degrees << endl;
      std::lock_guard<std::mutex> lock(mtx);

      if (rotating) {
        res.result = "Rotation already in progress.";
        ROS_WARN("Service Requested: Rotation already in progress.");
        return false;
      }

      if (!odom_got) {
        res.result = "No odometry data received yet.";
        ROS_ERROR("Service Requested: No odometry data received yet.");
        return false;
      }

      rotating = true;
    }

    ROS_INFO("Service Requested: Rotating %.2d degrees.", req.degrees);

    // Convert degrees to radians
    double target_angle = req.degrees * M_PI / 180.0;

    // Capture the initial yaw
    double initial_yaw;
    double target_yaw;
    {
      std::lock_guard<std::mutex> lock(mtx);
      initial_yaw = current_yaw;
      target_yaw = normalizeAngle(initial_yaw + target_angle);
    }

    // Determine rotation direction and speed
    double angular_speed = (req.degrees > 0) ? 0.5 : -0.5;

    ros::Rate rate(10); // 10 Hz
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = angular_speed;

    while (ros::ok()) {
      double error;
      {
        std::lock_guard<std::mutex> lock(mtx);
        error = smallestAngleDifference(target_yaw, current_yaw);
      }

      if (std::fabs(error) < 0.05) { // Threshold in radians (~2.86 degrees)
        vel_msg.angular.z = 0.0;
        cmd_vel_pub.publish(vel_msg);
        ROS_INFO("Service Completed: Rotation complete.");
        break;
      }

      cmd_vel_pub.publish(vel_msg);
      rate.sleep();
    }

    // Ensure the robot has stopped rotating
    vel_msg.angular.z = 0.0;
    cmd_vel_pub.publish(vel_msg);

    // Reset rotation state
    {
      std::lock_guard<std::mutex> lock(mtx);
      rotating = false;
    }

    res.result = "Rotation complete!";
    return true;
  }

  // Normalize angle to be within [-pi, pi]
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Compute the smallest difference between two angles
  double smallestAngleDifference(double target, double current) {
    double diff = target - current;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return diff;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_server");

  RotateServer rotate_server;

  // Use a multi-threaded spinner to handle callbacks concurrently
  ros::AsyncSpinner spinner(2); // Number of threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
