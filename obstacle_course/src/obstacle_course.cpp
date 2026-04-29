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
