#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int32.h"

void arrived_action(const std_msgs::Int32::ConstPtr& msg)
{
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    // set the namespace and id for this marker
    marker.ns = "markers";
    marker.id = 0;

    // set frame id and timestamp.
    marker.header.frame_id = "map"; // use absolute cordinates
    marker.header.stamp = ros::Time::now();

    // set marker type
    marker.type = shape;

    // set the scale of the marker
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // set the color = darkmagenta	#8B008B	rgb(139,0,139)
    marker.color.r = 0.545f;
    marker.color.g = 0.0f;
    marker.color.b = 0.545f;
    marker.color.a = 0.85f; // a little bit transparent

    marker.lifetime = ros::Duration();

    // publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    if (msg->data == 0) {
        // op started
        ROS_INFO("add marker at pickup");
        // set marker action
        marker.action = visualization_msgs::Marker::ADD;

        // set the pose of the marker. use the pick up goal pos
        marker.pose.position.x = -0.61;
        marker.pose.position.y = -2.65;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0.99;
        marker.pose.orientation.w = -0.13;

        marker_pub.publish(marker);
    } else if (msg->data == 2) {
        // reached pickup goal
        ROS_INFO("remove marker at pickup");
        // set marker action
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        ros::Duration(5).sleep();
    } else if (msg->data == 3) {
        // reached dropoff goal
        ROS_INFO("pub marker at dropoff");

        // set marker action
        marker.action = visualization_msgs::Marker::ADD;

        // set the pose of the marker. use the drop off goal pos
        marker.pose.position.x = -6.03;
        marker.pose.position.y = 2.1;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0.556;
        marker.pose.orientation.w = 0.83;

        // publish the marker
        marker_pub.publish(marker);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Subscriber check_arrival = n.subscribe("arrived_flag", 1000, arrived_action);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
