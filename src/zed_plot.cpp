// Copyright 2023 NC State Yoon Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/**
 * This node plots position and heading using ZED and S2 Lidar scan.
 */

#include <matplotlibcpp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "foxglove_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace plt = matplotlibcpp;

using namespace std::placeholders;

#define RAD2DEG 57.295779513

class MinimalPoseOdomSubscriber : public rclcpp::Node
{
public:
  MinimalPoseOdomSubscriber()
  : Node("zed_plot")
  {
    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();

    // Create pose subscriber
    mPoseSub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", qos, std::bind(&MinimalPoseOdomSubscriber::poseCallback, this, _1));

    // Create odom subscriber
    // mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
    //   "odom", qos, std::bind(&MinimalPoseOdomSubscriber::odomCallback, this, _1));

    // Create lidar scan subscriber
    mScanSub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&MinimalPoseOdomSubscriber::scanCallback, this, _1));

    // mPlotter = create_wall_timer(
    //   std::chrono::milliseconds(1000), std::bind(&MinimalPoseOdomSubscriber::plotter, this));

    // mScanPub = this->create_publisher<sensor_msgs::msg::LaserScan>("myScan", 10);
    mGridPub = create_publisher<nav_msgs::msg::OccupancyGrid>("myGrid", 10);

    mPosePub = create_publisher<geometry_msgs::msg::PoseStamped>("robotPose", 10);

    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    xArrow = std::vector<double>(2);
    yArrow = std::vector<double>(2);
    data = std::vector<int8_t>(640000, 0);
  }

protected:
  void plotter()
  {
//    plt::clf();
//    plt::plot(x, y, "-b");
//    plt::plot(xArrow, yArrow, "-r");
//    plt::scatter(scanX, scanY, 1);
//
//    plt::xlim(-5, 5);
//    plt::ylim(-5, 5);
//    plt::title("Zed2i Pos and Heading and S2 Scan");
//    plt::xlabel("x pos");
//    plt::ylabel("y pos");
//    plt::pause(0.01);

//    auto myGrid = nav_msgs::msg::OccupancyGrid();
//    myGrid.header.frame_id = "myGrid";
//    myGrid.info.width = 400;
//    myGrid.info.height = 400;
//    myGrid.info.resolution = 0.01;
//    myGrid.info.origin.position.x = -2;
//    myGrid.info.origin.position.y = -2;
//    std::vector<int8_t> data(160000, 100);
//    for (int i = 0; i < 40000; i++) {
//      data[i] = 0;
//    }
//    // data[0] = 100;
//    myGrid.data = data;
//    mGridPub->publish(myGrid);
        for (int i = 0; i < data.size(); i++) {
            if (data[i] > 0) {
                data[i] -= 1;
            }
        }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    int count = scan->scan_time / scan->time_increment;

    RCLCPP_INFO(
      get_logger(), "[SLLIDAR INFO]: I heard a laser scan %s[%d, %d]:\n",
      scan->header.frame_id.c_str(), count, scan->ranges.size());

    scanX.clear();
    scanY.clear();

    //std::vector<int8_t> data(160000, 0);

    for (int i = 0; i < count; i++) {
      float radian = scan->angle_min + scan->angle_increment * i;
      if (!isinf(scan->ranges[i])) {
        scanX.push_back(
          cos(radian - M_PI - (yawStart - yawCurr)) * scan->ranges[i] + xArrow[0] - xStart);
        scanY.push_back(
          sin(radian - M_PI - (yawStart - yawCurr)) * scan->ranges[i] + yArrow[0] - yStart);

        int row_index = (-scanX.back() + 2) / 0.01;
        int col_index = (scanY.back() + 2) / 0.01;
        if (row_index < 0 || row_index >= 800 || col_index < 0 || col_index >= 800) {
            continue;
        }
        int loc_index = (row_index * 800) + col_index;
        if (data[loc_index] < 100) data[loc_index] += 1;
        // data[loc_index] = 100;
      }
    }

    auto myGrid = nav_msgs::msg::OccupancyGrid();
    myGrid.header.frame_id = "myGrid";
    myGrid.info.width = 800;
    myGrid.info.height = 800;
    myGrid.info.resolution = 0.01;
    myGrid.info.origin.position.x = -2;
    myGrid.info.origin.position.y = -2;
    myGrid.data = data;
    mGridPub->publish(myGrid);

    // auto myScan = foxglove_msgs::msg::LaserScan();
    // mScanPub->publish(*scan);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Camera position in map frame
    double tx = msg->pose.position.x;
    double ty = msg->pose.position.y;
    double tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, yaw);

    RCLCPP_INFO(
      get_logger(),
      "R: %.2f P: %.2f Y: %.2f", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

    if (!first) {
      xStart = tx;
      yStart = ty;
      yawStart = yaw;
      first = true;
    }
    yawCurr = yaw;

    x.push_back(-(ty - yStart));
    y.push_back(tx - xStart);
    xArrow[0] = -(ty - yStart);
    xArrow[1] = -((ty - yStart) + 0.05 * sin(yaw));
    yArrow[0] = tx - xStart;
    yArrow[1] = (tx - xStart) + 0.05 * cos(yaw);

    auto robotPose = geometry_msgs::msg::PoseStamped();
    robotPose.header.frame_id = "robotPoseTopic";
    robotPose.pose.position.x = tx;
    robotPose.pose.position.y = ty;
    robotPose.pose.position.z = 0;
    robotPose.pose.orientation.x = q_new.getX();
    robotPose.pose.orientation.y = q_new.getY();
    robotPose.pose.orientation.z = q_new.getZ();
    robotPose.pose.orientation.w = q_new.getW();

    geometry_msgs::msg::TransformStamped robotPoseTransform;
    robotPoseTransform.transform.translation.x = tx;
    robotPoseTransform.transform.translation.y = ty;
    robotPoseTransform.transform.translation.z = 0;
    robotPoseTransform.transform.rotation.x = q_new.getX();
    robotPoseTransform.transform.rotation.y = q_new.getY();
    robotPoseTransform.transform.rotation.z = q_new.getZ();
    robotPoseTransform.transform.rotation.w = q_new.getW();
    robotPoseTransform.header.frame_id = "map";
    robotPoseTransform.child_frame_id = "robotPoseFrame";
    mTfBroadcaster->sendTransform(robotPoseTransform);

    mPosePub->publish(robotPose);

  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Camera position in map frame
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    RCLCPP_INFO(
      get_logger(),
      "Received odometry in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - "
      "Timestamp: %u.%u sec ",
      msg->header.frame_id.c_str(), tx, ty, tz, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

private:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> xArrow;
  std::vector<double> yArrow;
  std::vector<double> scanX;
  std::vector<double> scanY;
  bool first = false;
  double xStart;
  double yStart;
  double yawStart = 0;
  double yawCurr = 0;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mScanSub;
  rclcpp::TimerBase::SharedPtr mPlotter;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mScanPub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mGridPub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPosePub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  std::vector<int8_t> data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto pos_track_node = std::make_shared<MinimalPoseOdomSubscriber>();

  // matplotlib not thread safe so this was causing issues
  // std::thread worker(&MinimalPoseOdomSubscriber::plotter, pos_track_node);

  rclcpp::spin(pos_track_node);
  rclcpp::shutdown();

  return 0;
}
