#ifndef __YHS_CANCONTROL_NODE_H__
#define __YHS_CANCONTROL_NODE_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "yhs_can_interfaces/msg/io_cmd.hpp"
#include "yhs_can_interfaces/msg/auto_spd_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/auto_torque_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/remote_torque_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/odo_fb.hpp"

#define READ_PARAM(TYPE, NAME, VAR, VALUE)  \
  VAR = VALUE;                              \
  node->declare_parameter<TYPE>(NAME, VAR); \
  node->get_parameter(NAME, VAR);

namespace yhs
{
  class CanControl
  {

  public:
    CanControl(rclcpp::Node::SharedPtr);
    ~CanControl();

    bool run();
    void stop();

  private:
    rclcpp::Node::SharedPtr node_;

    std::string if_name_;
    int can_socket_;
    double wheel_base_;
    std::string odom_frame_;
    std::string base_link_frame_;
    bool tf_used_;
    std::thread thread_;
    std::mutex io_mutex_;
    std::mutex imu_mutex_;

    std::vector<int64_t> ultrasonic_number_;

    // io_cmd defaults loaded from params and used until topic updates arrive.
    yhs_can_interfaces::msg::IoCmd current_io_cmd_;

    // IMU state for odom fusion/fallback.
    rclcpp::Time last_imu_time_;
    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;

    // Internal odom integration state.
    double odom_x_;
    double odom_y_;
    double odom_theta_;
    rclcpp::Time odom_last_time_;

    rclcpp::Subscription<yhs_can_interfaces::msg::IoCmd>::SharedPtr io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::AutoSpdCtrlCmd>::SharedPtr auto_spd_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::AutoTorqueCtrlCmd>::SharedPtr auto_torque_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::RemoteTorqueCtrlCmd>::SharedPtr remote_torque_ctrl_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::TimerBase::SharedPtr io_timer_;

    rclcpp::Publisher<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
    rclcpp::Publisher<yhs_can_interfaces::msg::OdoFb>::SharedPtr odo_fb_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg);

    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    void io_timer_callback();

    void send_io_command();

    void auto_spd_ctrl_cmd_callback(const yhs_can_interfaces::msg::AutoSpdCtrlCmd::SharedPtr auto_spd_ctrl_cmd_msg);

    void auto_torque_ctrl_cmd_callback(const yhs_can_interfaces::msg::AutoTorqueCtrlCmd::SharedPtr auto_Torque_ctrl_cmd_msg);

    void remote_torque_ctrl_cmd_callback(const yhs_can_interfaces::msg::RemoteTorqueCtrlCmd::SharedPtr remote_Torque_ctrl_cmd_msg);

    bool wait_for_can_frame();

    void can_data_recv_callback();

    void publish_odom(const double velocity, const double steering);
  };

}

#endif
