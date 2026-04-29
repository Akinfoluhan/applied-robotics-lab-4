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
        "package://obstacle_course/meshes/obstacle_course.stl",
        pose);
}
