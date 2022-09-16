#include <ros/ros.h>
#include <PIASolver.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>

ros::NodeHandle *nh;
ros::Publisher *roomCenterPub;
PIASolver &solver = PIASolver::getInstance();

void LaserScanCallback(sensor_msgs::LaserScanPtr scan) {
    Point center = solver.solve(scan->ranges, scan->angle_increment);
    geometry_msgs::PointStamped centerMsg;
    centerMsg.point.x = center.x;
    centerMsg.point.y = center.y;
    centerMsg.point.z = 0;
    centerMsg.header.frame_id = "base_link";
    centerMsg.header.stamp = ros::Time::now();
    roomCenterPub->publish(centerMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "room_center_finder");
    *nh = ros::NodeHandle();
    ros::Subscriber laserScanSub = nh->subscribe<sensor_msgs::LaserScan>("ray/scan", 1, LaserScanCallback);
    *roomCenterPub = nh->advertise<geometry_msgs::PointStamped>("room_center", 1);
    ros::spin();
    return 0;
}