#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();

private:
    //helpers
    inline std::array<float, 3> body_to_world(float x, float y, float z) const
    {
        const float s = std::sin(init_yaw_), c = std::cos(init_yaw_);
        return {x * c - y * s, x * s + y * c, z};
    }
    static inline double wrap_pi(double a)
    {
        while (a > M_PI)
            a -= 2. * M_PI;
        while (a < -M_PI)
            a += 2. * M_PI;
        return a;
    }

    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f);
    void publish_trajectory_setpoint();

    void arm() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f); }
    void disarm();
    void load_csv_waypoints(const std::string &file);
    void vehicle_odom_callback(const VehicleOdometry::SharedPtr msg);

    //param
    static constexpr float DIST_THRESHOLD_METERS = 0.10f;
    static constexpr float SPEED_THRESHOLD_M_S = 0.20f;
    static constexpr double YAW_DIFF_THRESHOLD = 0.10;
    static constexpr double DWELL_TIME_SEC = 5.0;
    static constexpr double WP_TIMEOUT_SEC = 3.0;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t last_pose_ns_ = 0;

    std::vector<std::array<float, 4>> waypoints_;
    size_t wp_idx_ = 0;
    uint64_t setpt_cnt_ = 0;

    // pose & speed
    float x_ = 0, y_ = 0, z_ = 0;
    double yaw_ = 0;
    float speed_ = 0;
    rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time wp_start_time_{0, 0, RCL_ROS_TIME};
    float last_x_ = 0, last_y_ = 0, last_z_ = 0;

    //init ref
    bool yaw_init_ = false;
    float init_x_ = 0, init_y_ = 0, init_z_ = 0;
    double init_yaw_ = 0, last_cmd_yaw_ = 0;

    bool in_dwell_ = false;
    rclcpp::Time dwell_start_;
    bool landed_ = false;
    bool disarmed_ = false;

    bool ready_ = false;
    std::array<float, 3> mean_off_{0.f, 0.f, 0.f};
};

OffboardControl::OffboardControl() : Node("offboard_control")
{
    //publish subscribe
    odom_sub_ = create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS{},
        std::bind(&OffboardControl::vehicle_odom_callback, this, std::placeholders::_1));

    offboard_ctrl_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_sp_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_cmd_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    load_csv_waypoints("src/px4_offboard_control/src/path.csv");

    //100 ms loop
    timer_ = create_wall_timer(100ms, [this]
                               {
        publish_offboard_control_mode();

        if (setpt_cnt_ == 10) {
            RCLCPP_INFO(get_logger(), "Arm Command Sent");
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); //offboard mode
            arm();
        }
        ++setpt_cnt_;
        publish_trajectory_setpoint(); });

    RCLCPP_INFO(get_logger(), "OffboardControl node started.");
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1'000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_ctrl_pub_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t cmd, float p1, float p2)
{
    VehicleCommand vc{};
    vc.timestamp = get_clock()->now().nanoseconds() / 1'000;
    vc.param1 = p1;
    vc.param2 = p2;
    vc.command = cmd;
    vc.target_system = 1;
    vc.target_component = 1;
    vc.source_system = 1;
    vc.source_component = 1;
    vc.from_external = true;
    vehicle_cmd_pub_->publish(vc);
}

void OffboardControl::disarm()
{
    if (!disarmed_)
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f);
        disarmed_ = true;
    }
}

void OffboardControl::vehicle_odom_callback(const VehicleOdometry::SharedPtr msg)
{
    /* --- position --- */
    x_ = msg->position[0];
    y_ = msg->position[1];
    z_ = msg->position[2];

    /* --- velocity via finite‑difference --- */
    const int64_t stamp_ns = static_cast<int64_t>(msg->timestamp) * 1000LL;   // µs→ns
    if (last_pose_ns_ != 0) {                       // skip first message
        const double dt = (stamp_ns - last_pose_ns_) * 1e-9;                  // ns→s
        if (dt > 0.01) {
            const float dx = x_ - last_x_;
            const float dy = y_ - last_y_;
            const float dz = z_ - last_z_;
            speed_ = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;
        }
    }
    last_pose_ns_ = stamp_ns;
    last_x_ = x_;  last_y_ = y_;  last_z_ = z_;

    /* --- yaw --- */
    tf2::Quaternion q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);

    if (!yaw_init_)
    {
        init_x_ = x_;
        init_y_ = y_;
        init_z_ = z_;
        init_yaw_ = yaw_;
        last_cmd_yaw_ = yaw_;
        yaw_init_ = true;
        RCLCPP_INFO(get_logger(),
                    "Init ref ↗ yaw=%.1f°  pos=(%.2f,%.2f,%.2f)",
                    init_yaw_ * 180. / M_PI, init_x_, init_y_, init_z_);
    }
}

/* -------------------------------------------------------------------------- */

void OffboardControl::publish_trajectory_setpoint()
{
    if (!yaw_init_ || waypoints_.empty())
    {
        return;
    }

    TrajectorySetpoint sp{};
    sp.timestamp = get_clock()->now().nanoseconds() / 1'000;

    /* ---------------- normal WP handling ---------------- */
    if (wp_idx_ < waypoints_.size())
    {

        const auto wp = waypoints_[wp_idx_];
        const auto pos = body_to_world(wp[0], wp[1], wp[2]);
        const double cmd_yaw = init_yaw_ + wp[3];

        static size_t last_wp_seen = SIZE_MAX;
        if (last_wp_seen != wp_idx_) {
            last_wp_seen   = wp_idx_;
            wp_start_time_ = get_clock()->now();
        }

        sp.position = {pos[0], pos[1], pos[2]};
        sp.yaw = cmd_yaw;

        /* check arrival ---------------------------------------------------- */
        const float dx = x_ - init_x_ - pos[0];
        const float dy = y_ - init_y_ - pos[1];
        const float dz = z_ - init_z_ - pos[2];
        const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double yaw_err = std::fabs(wrap_pi(cmd_yaw - last_cmd_yaw_));

        if ((get_clock()->now() - wp_start_time_).seconds() > WP_TIMEOUT_SEC) {
            RCLCPP_WARN(get_logger(),
                        "WP %zu timed‑out after %.1f s → skipping",
                        wp_idx_, WP_TIMEOUT_SEC);
            ++wp_idx_;
            in_dwell_ = false;
            /* reset timer for the new WP */
            last_wp_seen = SIZE_MAX;
            return;          // publish current sp and wait for next loop tick
        }

        if (dist < DIST_THRESHOLD_METERS)
        {

            const bool yaw_changed = yaw_err > YAW_DIFF_THRESHOLD;

            if (!yaw_changed)
            {
                RCLCPP_INFO(get_logger(), "WP %zu reached", wp_idx_);
                ++wp_idx_;
                last_cmd_yaw_ = cmd_yaw;
                in_dwell_ = false;
            }
            else
            {
                if (!in_dwell_)
                {
                    dwell_start_ = get_clock()->now();
                    in_dwell_ = true;
                }
                if ((get_clock()->now() - dwell_start_).seconds() > DWELL_TIME_SEC)
                {
                    RCLCPP_INFO(get_logger(),
                                "WP %zu complete (after yaw dwell)", wp_idx_);
                    ++wp_idx_;
                    last_cmd_yaw_ = cmd_yaw;
                    in_dwell_ = false;
                }
            }
        }
    }
    else if (!landed_)
    {
        /* ---------------- landing phase ----------------------------------- */
        const auto last = waypoints_.back();
        const auto p = body_to_world(last[0], last[1], 0.f);
        sp.position = {p[0], p[1], 0.f};
        sp.yaw = init_yaw_ + last[3];

        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        landed_ = true;
        RCLCPP_INFO(get_logger(), "LAND command sent");
    }

    traj_sp_pub_->publish(sp);
}

/* -------------------------------------------------------------------------- */

void OffboardControl::load_csv_waypoints(const std::string &file)
{
    std::ifstream fs(file);
    if (!fs.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Unable to open CSV '%s'", file.c_str());
        return;
    }
    std::string line;
    bool header = true;
    while (std::getline(fs, line))
    {
        if (header)
        {
            header = false;
            continue;
        }
        std::stringstream ss(line);
        std::string tok;
        std::vector<float> v;
        while (std::getline(ss, tok, ','))
            v.push_back(std::stof(tok));
        if (v.size() == 4)
            waypoints_.push_back({v[0], v[1], v[2], v[3]});
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());
}

/* -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    std::cout << "Starting OffboardControl node …\n";
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ); // flush std::cout immediately
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}