#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // set initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok()) {
        visualization_msgs::Marker marker;
        // set frame id and timestamp.
        marker.header.frame_id = "map"; // use absolute cordinates
        marker.header.stamp = ros::Time::now();

        // set the namespace and id for this marker
        marker.ns = "markers";
        marker.id = 0;

        // set marker type
        marker.type = shape;

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

        // set the scale of the marker
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // set the color = darkmagenta	#8B008B	rgb(139,0,139)
        marker.color.r = 0.545f;
        marker.color.g = 0.0f;
        marker.color.b = 0.545f;
        marker.color.a = 0.85f; // a little bit transparent

        marker.lifetime = ros::Duration(); // marker will live for 5 sec

        // publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        ros::Duration(5).sleep(); // pause for 5 sec

        // set the pose of the marker. use the drop off goal pos
        marker.pose.position.x = -6.03;
        marker.pose.position.y = 2.1;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0.556;
        marker.pose.orientation.w = 0.83;

        // publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();
    }
}
