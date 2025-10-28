#include "ros_interface.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include "imu.h"
#include "odometry.h"
#include "motor_control.h"

rcl_publisher_t imu_pub, odom_pub;
rcl_subscription_t cmd_vel_sub;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;

IMUManager imu;

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *cmd = (const geometry_msgs__msg__Twist *)msgin;
    float vx = cmd->linear.x;
    float vy = cmd->linear.y;
    float wz = cmd->angular.z;
    control_motors(vx, vy, wz);  // ここに指令を渡す
}

void ros_setup() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu");

    rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
}

void ros_loop(float dt) {
    imu.update(&imu_msg, dt);
    rcl_publish(&imu_pub, &imu_msg, NULL);

    updateOdometry(&odom_msg, dt);
    rcl_publish(&odom_pub, &odom_msg, NULL);

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
