#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_02_aruco_marker/ArucoDetector2DAction.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main (int argc, char **argv) {
    ros::init(argc, argv, "aruco_client");

    actionlib::SimpleActionClient<task_02_aruco_marker::ArucoDetector2DAction> ac("localization", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    cv::Mat img;
    // img = cv::imread("/home/ashik/ros_workspace/catkin_ws/src/pr_assignment/task_02_aruco_marker/images/markers_6X6.png");
    // img = cv::imread("/home/ashik/Downloads/Brush+20-50-50.jpg"); // Pose2D X:0.18 | Y:-0.76 | th:-0.35
    // img = cv::imread("/home/ashik/Downloads/Brush-20+50+50.jpg"); // Pose2D X:-0.40 | Y:-0.95 | th:0.35 
    img = cv::imread("/home/ashik/Downloads/Brush+180+0+0.jpg"); // Pose2D X:-0.01 | Y:-0.81 | th:-0.00

    std_msgs::Header header;
    header.seq      = 10;
    header.stamp    = ros::Time::now();

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image imgMsg;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(imgMsg);

    ROS_INFO("Action server started, sending goal.");

    task_02_aruco_marker::ArucoDetector2DGoal goal;
    goal.image = imgMsg;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        task_02_aruco_marker::ArucoDetector2DResult::ConstPtr result = ac.getResult();

        ROS_INFO("Pose2D X:%.2f | Y:%.2f | th:%.2f ", result->pose.x, result->pose.y, result->pose.theta);
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    } else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}