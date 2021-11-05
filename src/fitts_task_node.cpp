//
// Created by tucker on 11/3/21.
//

#include <polaris_sensor/polaris_sensor.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <iostream>

void callback(sensor_msgs::PointCloud pcl)
{
    std::cout << "Callback" << std::endl;
}

void targets_callback(geometry_msgs::PoseArray arr)
{
    std::cout << arr.poses[0].position.x << ", " << arr.poses[0].position.y << ", " << \
    arr.poses[0].position.z << std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fitts_task");
    ros::NodeHandle nh;
    std::cout << "Running fitts task" << std::endl;

//     Subscribe to the polaris sensor data
    ros::Subscriber polaris_cloud_sub = nh.subscribe<sensor_msgs::PointCloud>("polaris_sensor/targets_cloud", 1, callback);
    ros::Subscriber polaris_targets_sub = nh.subscribe<geometry_msgs::PoseArray>("polaris_sensor/targets", 1, targets_callback);

    ros::Rate loop_rate(100);
    ros::spin();

    // Begin fitts task
    // Begin with calibration
    // Prepare fitts task
    // Execute task and record data
    // Save data
}