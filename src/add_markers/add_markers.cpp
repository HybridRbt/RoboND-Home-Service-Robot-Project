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
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // set the color = darkmagenta	#8B008B	rgb(139,0,139)
    marker.color.r = 0.545f;
    marker.color.g = 0.0f;
    marker.color.b = 0.545f;
    marker.color.a = 0.85f; // a little bit transparent

    marker.lifetime = ros::Duration();
}

void Marker_drawer::drawAtPickUp()
{
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
}

void Marker_drawer::drawAtDropoff()
{
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

    marker_pub.publish(marker);
}

void Marker_drawer::arrived_action(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0)
    {
        // op started
        ROS_INFO("add marker at pickup");
        drawAtPickUp();
    }
    else if (msg->data == 1)
    {
        // reached pickup goal
        ROS_INFO("remove marker at pickup");

        // set marker action
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        ros::Duration(5).sleep();
    }
    else if (msg->data == 2)
    {
        // reached dropoff goal
        ROS_INFO("pub marker at dropoff");
        drawAtDropoff();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ROS_INFO("node inited");
    ros::start();

    ros::NodeHandle n;

    Marker_drawer drawer;

    ros::Subscriber check_arrival = n.subscribe("arrived_flag",
                                                1000,
                                                &Marker_drawer::arrived_action,
                                                &drawer);

    drawer.setPub(&n);

    ros::spin();
}
