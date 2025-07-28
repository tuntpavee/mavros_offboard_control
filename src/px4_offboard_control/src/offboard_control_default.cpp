#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <atomic>
#include <cmath>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint {
    float x, y, z, yaw;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // Publishers
        offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Initial state
        setpoint_counter_ = 0;
        current_index_ = 0;
        ticks_ = 0;
        yaw_ = 1.57f;
        mode_state_ = TAKEOFF;
        disarmed_ = false;

        // Load waypoints
        read_csv_and_compute_yaw("src/px4_offboard_control/src/path.csv");

        // Timer
        timer_ = create_wall_timer(100ms, [this]() {
            if (setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Offboard mode
                arm();
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            if (setpoint_counter_ < 11) {
                setpoint_counter_++;
            }
        });
    }

private:
    enum ModeState { TAKEOFF, POSITIONING, HOVERING, YAWING , LANDING };
    ModeState mode_state_;

    // ROS2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Internal state
    std::atomic<uint64_t> timestamp_;
    std::vector<Waypoint> waypoints_;
    
    size_t current_index_;
    uint64_t setpoint_counter_;
    int ticks_;
    float yaw_;
    bool disarmed_;

    // Helper Methods
    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void read_csv_and_compute_yaw(const std::string &filename);
};

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command sent");
}

void OffboardControl::disarm()
{
    if (!disarmed_) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(get_logger(), "Disarm command sent");
        disarmed_ = true;
    }
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_control_mode_pub_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;

    static bool landing_cmd_sent = false ;
    static int landing_tick_counter = 0 ;

    if (mode_state_ == LANDING) {
        if(!landing_cmd_sent){
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            RCLCPP_INFO(get_logger(),"Landing");
            landing_cmd_sent = true ;
            timer_->cancel();
            return ;
        }

        if (landing_tick_counter++ >= 100){
            disarm();
            timer_->cancel();
        }
        return ;
    }

    if (current_index_ >= waypoints_.size()) {
        mode_state_ = LANDING ;
        return;
    }

    const Waypoint &wp = waypoints_[current_index_];

    if (mode_state_ == TAKEOFF) {
        msg.position = {0.0f, 0.0f, -wp.z};
        msg.yaw = wp.yaw;
        trajectory_setpoint_pub_->publish(msg);

        if (++ticks_ >= 100) {
            mode_state_ = POSITIONING;
            ticks_ = 0;
        }   

    } 

    else {
        msg.position = {wp.x , wp.y , -wp.z};
        msg.yaw = wp.yaw ;
    
    
        if (++ticks_ >= 50) {
            current_index_++;
            

            if (current_index_ >= waypoints_.size()){
                mode_state_ = LANDING ;
            }

            else {
                mode_state_ = POSITIONING ;
            }

            ticks_ = 0;
        }

        trajectory_setpoint_pub_->publish(msg);
    }

}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
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

void OffboardControl::read_csv_and_compute_yaw(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s", filename.c_str());
        return;
    }

    std::string line;
    std::vector<Waypoint> raw;

    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string sx, sy, sz, syaw;
        std::getline(ss, sx, ',');
        std::getline(ss, sy, ',');
        std::getline(ss, sz, ',');
        std::getline(ss, syaw, ',');

        try {
            Waypoint wp{std::stof(sx), std::stof(sy), std::stof(sz), std::stof(syaw)};
            raw.push_back(wp);
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "Failed to parse line: %s", line.c_str());
        }
    }

    waypoints_ = raw;
    RCLCPP_INFO(get_logger(), "Loaded %ld waypoints from CSV", waypoints_.size());
}

int main(int argc, char *argv[])
{
    std::cout << "Starting Offboard Control Node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
