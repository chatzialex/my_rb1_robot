#include "my_rb1_ros/Rotate.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include "ros/spinner.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

class RotateServiceServer {
public:
  RotateServiceServer(const std::string &name = "rotate_robot")
      : service_name_{name},
        twist_publisher_{
            node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1000)},
        odometry_subscriber_{node_handle_.subscribe(
            "/odom", 1000, &RotateServiceServer::odomCallback, this)},
        service_server_{node_handle_.advertiseService(
            service_name_, &RotateServiceServer::service_callback, this)} {
    ROS_INFO("rotate_robot service server succesfully launched.");
  }

private:
  static constexpr double kAngularVelocity{0.5};
  static constexpr double kRateHz{10};
  static constexpr double kTimeoutSec{20};
  static constexpr double kRadToDegreesFactor{180 / 3.1416};

  std::string service_name_{};
  ros::NodeHandle node_handle_{};
  ros::Rate rate{kRateHz};
  ros::Publisher twist_publisher_{};
  ros::Subscriber odometry_subscriber_{};
  ros::ServiceServer service_server_{};
  double rotation_{};
  nav_msgs::Odometry odometry_{};

  bool service_callback(my_rb1_ros::Rotate::Request &req,
                        my_rb1_ros::Rotate::Response &res);

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

bool RotateServiceServer::service_callback(my_rb1_ros::Rotate::Request &req,
                                           my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("%s: Initiating a %i degree rotation.", service_name_.c_str(),
           req.degrees);
  const double initial_rotation{rotation_};
  bool success{false};
  geometry_msgs::Twist twist{};
  double elapsed_time{0.0};
  const int desired_rotation_sign{req.degrees >= 0 ? 1 : -1};
  twist.angular.z = desired_rotation_sign * kAngularVelocity;

  while (elapsed_time <= kTimeoutSec) {
    const double current_rotation_deg{(rotation_ - initial_rotation) *
                                      kRadToDegreesFactor};
    std::cout << "rotation_:" << rotation_ << std::endl;
    std::cout << "initial_rotation:" << initial_rotation << std::endl;
    std::cout << "current_rotation_deg:" << current_rotation_deg << std::endl;
    ROS_DEBUG("%s: %f seconds elapsed, rotated by %f degrees",
              service_name_.c_str(), elapsed_time, current_rotation_deg);
    if (desired_rotation_sign * (current_rotation_deg - req.degrees) >= 0) {
      success = true;
      break;
    }
    twist_publisher_.publish(twist);
    rate.sleep();
    elapsed_time += 1 / kRateHz;
  }

  twist.angular.z = 0.0;
  twist_publisher_.publish(twist);
  res.result = success ? "Rotation successful."
                       : "Failed to rotate by the requested angle.";
  ROS_INFO("%s: %s", service_name_.c_str(), res.result.c_str());
  return success;
}

void RotateServiceServer::odomCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion q{};
  double yaw{};
  double pitch{};
  double roll{};

  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  rotation_ = yaw;
  /* ROS_INFO("[odom_callback] rotation = %f degrees",
           rotation_ * kRadToDegreesFactor); */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");

  RotateServiceServer rotate_service_server{};

  // need two threads because otherwise the service callback
  // blocks the subscriber callback

  ros::AsyncSpinner spinner{2};
  spinner.start();
  ros::waitForShutdown();

  return 0;
}