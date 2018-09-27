#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int32.h"

class Marker_drawer {
private:

    ros::Publisher marker_pub;
    uint32_t shape;
    visualization_msgs::Marker marker;

public:

    Marker_drawer();
    void arrived_action(const std_msgs::Int32::ConstPtr& msg);
    void setPub(ros::NodeHandle *n);
    void setDrawer();
    void drawAtPickUp();
    void drawAtDropoff();
    void drawMarker();
};

Marker_drawer::Marker_drawer()
{
    setDrawer();
    ROS_INFO("drawer inited");
}

void Marker_drawer::setPub(ros::NodeHandle *n)
{
    ROS_INFO("pub is set");
    marker_pub =
        n->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    drawAtPickUp();
}

void Marker_drawer::setDrawer()
{
    shape = visualization_msgs::Marker::CUBE;

    // set the namespace and id for this marker
    marker.ns = "markers";
    marker.id = 0;

    // set frame id and timestamp.
    marker.header.frame_id = "map"; // use absolute cordinates
    marker.header.stamp    = ros::Time::now();

    // set marker type
    marker.type = shape;

    // set the scale of the marker
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    // set the color = red
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.95f; // a little bit transparent

    marker.lifetime = ros::Duration();
}

void Marker_drawer::drawAtPickUp()
{
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
}

void Marker_drawer::drawAtDropoff()
{
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
}

void Marker_drawer::drawMarker() {
    marker_pub.publish(marker);
}

void Marker_drawer::arrived_action(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 1)
    {
        // op started
        drawAtPickUp();
    }
    else if (msg->data == 2)
    {
        // reached pickup goal
        ROS_INFO("remove marker at pickup");

        // set marker action
        marker.action = visualization_msgs::Marker::DELETE;

        ros::Duration(5).sleep();
    }
    else if (msg->data == 3)
    {
        // reached dropoff goal
        drawAtDropoff();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ROS_INFO("node inited");
    ros::start();
    ros::Rate r(1);
    
    ros::NodeHandle n;

    Marker_drawer drawer;
    drawer.setPub(&n);

    ros::Subscriber check_arrival = n.subscribe("arrived_flag",
                                                1000,
                                                &Marker_drawer::arrived_action,
                                                &drawer);

    while (ros::ok()) {
        ros::spinOnce();
        drawer.drawMarker();
        r.sleep();
    }
}
