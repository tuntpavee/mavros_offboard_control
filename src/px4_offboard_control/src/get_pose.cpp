#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardWithMavrosPose : public rclcpp::Node
{
public:
    OffboardWithMavrosPose() : Node("offboard_with_mavros_pose")
    {
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_   = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_       = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&OffboardWithMavrosPose::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardWithMavrosPose::timer_callback, this));
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int setpoint_counter_ = 0;
    bool mode_sent_ = false;
    bool arm_sent_ = false;

    void timer_callback()
    {
        // Always send OffboardControlMode + Setpoint first
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        setpoint_counter_++;

        // Wait until a few setpoints have been sent
        if (setpoint_counter_ == 20 && !mode_sent_) {
            set_mode_offboard();
            mode_sent_ = true;
            RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD mode command");
        }

        if (setpoint_counter_ == 30 && !arm_sent_) {
            arm();
            arm_sent_ = true;
            RCLCPP_INFO(this->get_logger(), "Sent ARM command");
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const auto &p = msg->pose.position;
        RCLCPP_INFO(this->get_logger(), "Current position (mavros): x=%.2f, y=%.2f, z=%.2f", p.x, p.y, p.z);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {0.0f, 0.0f, 4.0f};  // Target 4m altitude
        msg.yaw = 0.0f;
        trajectory_setpoint_pub_->publish(msg);
    }

    void set_mode_offboard()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); // PX4 OFFBOARD mode
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f); // ARM
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardWithMavrosPose>());
    rclcpp::shutdown();
    return 0;
}
