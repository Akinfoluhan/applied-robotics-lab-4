#ifndef OBSTACLE_COURSE
#define OBSTACLE_COURSE

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <sensor_msgs/msg/joy.hpp>

class ObstacleCourse : public rclcpp::Node
{
public:
    /**
     * Main constructor
     * sets up the publisher for the /display_planned_path topic
     * sets up the subscriber for the /joy topic
     */
    ObstacleCourse();

    /**
     * Destructor
     */
    ~ObstacleCourse() = default;

    /**
     * main robot motion code
     */
    void mainRobotMotion();

    /**
     * initializes the movegroup
     * sets the max planning time
     * sets the max number of planning attempts
     */
    void initMoveGroup();

    /**
     * adds mesh collision objects to the planning scene
     */
    void addMeshObstacles();

    /**
     * plan a path to the input joint angles
     * if successful, visualize the path plan by publishing to the /display_planned_path topic
     * @param joints The target joint angles
     * @return true if the plan succeeds
     */
    bool planMotionAndVisualize(std::vector<double> & joints);

    /**
     * execute the computed path plan
     */
    void moveJoints();

private:

    /**
     * callback function for game controller
     * executes motion only when A is pressed and was not pressed in the previous message
     * @param msg The incoming joy message
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * adds one mesh collision object to the planning scene
     * @param obstacle_name The name of the obstacle
     * @param mesh_resource The package path to the STL file
     * @param pose The pose of the obstacle in the world frame
     */
    void addMeshObstacle(
        const std::string & obstacle_name,
        const std::string & mesh_resource,
        const geometry_msgs::msg::Pose & pose);

    // planning scene interface
    moveit::planning_interface::PlanningSceneInterface psi_;

    // path plan object
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    // move group object
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // starting joint angles
    std::vector<double> start_joints_;

    // goal joint angles
    std::vector<double> goal_joints_;

    // publisher for /display_planned_path topic
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;

    // subscriber for /joy topic
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // tracks whether a valid path has been planned and can be executed
    bool plan_ready_;

    // tracks the previous state of the A button to prevent repeated execution
    bool previous_a_button_;
};

#endif // OBSTACLE_COURSE
