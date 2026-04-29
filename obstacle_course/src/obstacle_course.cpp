#include "obstacle_course/obstacle_course.h"
#include <chrono>
#include <cmath>


ObstacleCourse::ObstacleCourse() 
    : Node("obstacle_course")
{
    // set up publisher for /display_planned_path topic
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", 10);
    
    // setup subscriber for /joy topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&ObstacleCourse::joyCallback, this, std::placeholders::_1));

    // set initial starting joint angles
    start_joints_ = {-M_PI / 2.0, M_PI / 4.0, 0.0, 0.0, 0.0, 0.0};

    // set goal joint angles
    goal_joints_ = {M_PI / 2.0, M_PI / 2.0, 0.0, 0.0, 0.0, 0.0};

}


void ObstacleCourse::mainRobotMotion()
{
    rclcpp::sleep_for(std::chrono::seconds(1));

    // initialize movegroup
    initMoveGroup();

    rclcpp::sleep_for(std::chrono::seconds(2));

    // add in mesh collision objects
    addMeshObstacles();

    rclcpp::sleep_for(std::chrono::seconds(2));

    // plan motion to goal config
    bool plan_success = planMotionAndVisualize(goal_joints_);

    if (plan_success)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Path plan displayed in RVIZ. Press A on the game controller to execute.");
    }
    else
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Path planning failed. Nothing will be executed.");
    }

    // keep node alive so joy callback can receive button presses
    while (rclcpp::ok() && plan_ready_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void ObstacleCourse::initMoveGroup()
{
    // setup move group with name "rv_3s_arm"
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        "rv_3s_arm");

    // set planning attempts to 1000
    move_group_->setNumPlanningAttempts(1000);

    // set planning time to 300 seconds
    move_group_->setPlanningTime(300.0);
}


void ObstacleCourse::addMeshObstacle(
    const std::string & obstacle_name,
    const std::string & mesh_resource,
    const geometry_msgs::msg::Pose & pose)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = obstacle_name;

    std::unique_ptr<shapes::Mesh> mesh(
        shapes::createMeshFromResource(mesh_resource));

    if (!mesh)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load mesh");
        return;
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh.get(), shape_msg);

    shape_msgs::msg::Mesh mesh_msg =
        boost::get<shape_msgs::msg::Mesh>(shape_msg);

    obj.meshes.push_back(mesh_msg);
    obj.mesh_poses.push_back(pose);
    obj.operation = obj.ADD;

    psi_.applyCollisionObject(obj);

    RCLCPP_INFO(this->get_logger(), "Mesh obstacle added");
}


void ObstacleCourse::addMeshObstacles()
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;

    addMeshObstacle(
        "obstacle_course",
        "package://obstacle_course/meshes/obstacle_mitsubishi_meters.stl",
        pose);
}


bool ObstacleCourse::planMotionAndVisualize(std::vector<double> & joints)
{
    if (!move_group_)
    {
        RCLCPP_ERROR(this->get_logger(), "Move group has not been initialized");
        return false;
    }

    // set movegroup to target joints
    move_group_->setJointValueTarget(joints);

    // plan motion
    if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        // setup message of type moveit_msgs::msg::DisplayTrajectory
        moveit_msgs::msg::DisplayTrajectory display_msg;

        display_msg.trajectory_start = plan_.start_state_;
        display_msg.trajectory.push_back(plan_.trajectory_);

        // publish trajectory to /display_planned_path topic
        display_pub_->publish(display_msg);

        plan_ready_ = true;

        RCLCPP_INFO(this->get_logger(), "Path planning succeeded");
        return true;
    }
    else
    {
        plan_ready_ = false;

        RCLCPP_ERROR(this->get_logger(), "Path planning failed");
        return false;
    }
}


void ObstacleCourse::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // button A press detection
    bool a_pressed = (msg->buttons[0] == 1);

    // only execute the plan on the rising edge of button A
    if (a_pressed && !previous_a_button_)
    {
        if (plan_ready_)
        {
            moveJoints();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No valid plan ready");
        }
    }

    // store current button state for next callback
    previous_a_button_ = a_pressed;
}

void ObstacleCourse::moveJoints()
{
    move_group_->execute(plan_);

    plan_ready_ = false;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ObstacleCourse>();

    node->mainRobotMotion();

    rclcpp::shutdown();

    return 0;
}
