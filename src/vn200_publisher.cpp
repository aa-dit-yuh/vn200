#include <vn200_node.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "IMU");

    vectornav::VN200* vn200 = new vectornav::VN200();

    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu > ("imu", 30, true);

    sensor_msgs::Imu imu_data;

    ros::Rate publisher_rate(10);

    ROS_INFO("Start publishing...\n");

    while (ros::ok()) {

        ros::spinOnce();

        imu_data = vn200->getIMUData();

        imu_publisher.publish(imu_data);

        publisher_rate.sleep();

    }
    return 0;
}
