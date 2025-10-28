#ifndef ODOMETRY_H
#define ODOMETRY_H

void update_odometry(nav_msgs_Odometry &odom_msg,float v, float w ,float dt);
#endif // ODOMETRY_H