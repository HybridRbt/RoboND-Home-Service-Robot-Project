#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for sending goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait 5 seconds for the move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pickup_goal;
    move_base_msgs::MoveBaseGoal dropoff_goal;

    // set up the frame parameters
    pickup_goal.target_pose.header.frame_id = "map"; // this means the goal position will be absolute
    pickup_goal.target_pose.header.stamp = ros::Time::now();

    dropoff_goal.target_pose.header.frame_id = "map"; // this means the goal position will be absolute
    dropoff_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup_goal.target_pose.pose.position.y = 1.0;
    pickup_goal.target_pose.pose.orientation.w = 1.0;

    dropoff_goal.target_pose.pose.position.y = -1.0;
    dropoff_goal.target_pose.pose.orientation.w = 1.0;

    // send the pickup goal position and orientation for the robot to reach
    ROS_INFO("Sending pickup goal");
    ac.sendGoal(pickup_goal);

    // wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot has reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to the pick up goal");
    } else {
        ROS_INFO("The base failed to move to the pick up goal for some reason");
    }

    // wait 5 seconds before moving to dropoff goal
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("waiting 5 sec before moving to dropoff");
    }

    // send the dropoff goal position and orientation for the robot to reach
    ROS_INFO("Sending dropoff goal");
    ac.sendGoal(dropoff_goal);

    // wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot has reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to the drop off goal");
    } else {
        ROS_INFO("The base failed to move to drop off goal for some reason");
    }

    return 0;
}
