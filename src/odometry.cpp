#include "odometry.h"
#include <nav_msgs/msg/odometry.h>
#include <math.h>

float robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
const float WHEEL_BASE = 0.4; // [m] タイヤ間距離

void updateOdometryFromWheels(nav_msgs__msg__Odometry* msg,
                              float vL, float thetaL,
                              float vR, float thetaR,
                              float dt)
{
    float vx = (vL * cos(thetaL) + vR * cos(thetaR)) / 2.0;
    float vy = (vL * sin(thetaL) + vR * sin(thetaR)) / 2.0;
    float w  = (vR * sin(thetaR) - vL * sin(thetaL)) / WHEEL_BASE;

    // ロボット座標→世界座標系変換
    float cos_yaw = cos(robot_yaw);
    float sin_yaw = sin(robot_yaw);

    robot_x += (vx * cos_yaw - vy * sin_yaw) * dt;
    robot_y += (vx * sin_yaw + vy * cos_yaw) * dt;
    robot_yaw += w * dt;

    msg->pose.pose.position.x = robot_x;
    msg->pose.pose.position.y = robot_y;
    msg->pose.pose.orientation.z = sin(robot_yaw/2);
    msg->pose.pose.orientation.w = cos(robot_yaw/2);

    msg->twist.twist.linear.x = vx;
    msg->twist.twist.linear.y = vy;
    msg->twist.twist.angular.z = w;
}
