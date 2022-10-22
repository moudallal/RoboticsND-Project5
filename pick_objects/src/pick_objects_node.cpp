#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

// Define empty goal
double goal[3] = {0, 0, 0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void getGoal(const visualization_msgs::Marker::ConstPtr &msg)
{
    goal[0] = msg->pose.position.x;
    goal[1] = msg->pose.position.y;
    goal[2] = msg->pose.orientation.w;
    ROS_INFO("Update goal %f %f %f", goal[0], goal[1], goal[2]);
}

void sendGoal(MoveBaseClient &ac, double x, double y, double orientation)
{
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = orientation;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal ...");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, reached the goal!");
    else
        ROS_INFO("The base failed to reach the goal for some reason.");
}

int main(int argc, char **argv)
{
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    ros::NodeHandle n;
    ros::Subscriber marker_sub = n.subscribe("visualization_marker", 10, getGoal);

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::spinOnce();

    sendGoal(ac, goal[0], goal[1], goal[2]);

    // Simulate pickup by waiting for five seconds
    sleep(5);

    ros::spinOnce();

    sendGoal(ac, goal[0], goal[1], goal[2]);

    ROS_INFO("Reached the drop off zone");

    return 0;
}