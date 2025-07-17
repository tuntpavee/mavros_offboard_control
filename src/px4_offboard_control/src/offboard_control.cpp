#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
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
    OffboardControl()
    : Node("offboard_control")
    {
        offboard_ctrl_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_sp_pub_       = create_publisher<TrajectorySetpoint> ("/fmu/in/trajectory_setpoint", 10);
        vehicle_cmd_pub_   = create_publisher<VehicleCommand>     ("/fmu/in/vehicle_command",    10);

        odom_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::pose_cb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&OffboardControl::loop, this));

        load_csv_waypoints("/home/cone/ws_offboard_control/src/px4_offboard_control/src/path.csv");
    }

private:
    /* ================= Tunables ================= */
    static constexpr float  DIST_THRESHOLD_METERS = 1.0f;   // pos gate
    static constexpr float  SPEED_THRESHOLD_M_S   = 0.20f;   // max speed to start dwell timer
    static constexpr double YAW_DIFF_THRESHOLD    = 0.10;    // rad, ≈6°
    static constexpr double DWELL_TIME_SEC        = 5.0;     // yaw-dwell duration

    /* ============ ROS 2 handles ============ */
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      vehicle_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr                      timer_;

    /* ============ Mission state ============ */
    std::vector<std::array<float,4>> waypoints_;
    size_t    wp_idx_     = 0;
    uint64_t  setpt_cnt_  = 0;
    bool      landed_     = false;

    /* ============ Vehicle state (ENU) ============ */
    float   x_ = 0, y_ = 0, z_ = 0;
    double  yaw_ = 0;                 // rad, ENU frame
    float   speed_ = 0;
    bool    yaw_init_ = false;
    double  init_yaw_ = 0.0;
    double  last_cmd_yaw_ = 0.0;

    /* prev pose for speed estimate */
    float        last_x_ = 0, last_y_ = 0, last_z_ = 0;
    rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

    /* ============ Dwell tracking ============ */
    bool        in_dwell_ = false;
    rclcpp::Time dwell_start_;

    /* -------------- helpers -------------- */
    static double wrap_pi(double a)
    {
        while (a >  M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

    void publish_vehicle_cmd(uint16_t cmd, float p1 = 0, float p2 = 0)
    {
        VehicleCommand m{};
        m.timestamp = now().nanoseconds() / 1e3;
        m.param1 = p1;
        m.param2 = p2;
        m.command = cmd;
        m.target_system = m.source_system = 1;
        m.target_component = m.source_component = 1;
        m.from_external = true;
        vehicle_cmd_pub_->publish(m);
    }
    void arm() { publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1); }

    std::array<float,3> body_to_world(float x, float y, float z) const
    {
        float s = std::sin(init_yaw_), c = std::cos(init_yaw_);
        return { x*c - y*s, x*s + y*c, z };
    }

    /* ------------ Pose callback ------------ */
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        /* position */
        x_ = msg->pose.position.x;
        y_ = msg->pose.position.y;
        z_ = msg->pose.position.z;

        /* speed (finite difference) */
        rclcpp::Time stamp(msg->header.stamp);          // convert to rclcpp::Time
        if (last_pose_time_.nanoseconds() != 0) {
            double dt = (stamp - last_pose_time_).seconds();
            if (dt > 0.01) {
                float dx = x_ - last_x_;
                float dy = y_ - last_y_;
                float dz = z_ - last_z_;
                speed_ = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;
            }
        }
        last_x_ = x_;
        last_y_ = y_;
        last_z_ = z_;
        last_pose_time_ = stamp;

        /* yaw */
        tf2::Quaternion q(msg->pose.orientation.x,
                          msg->pose.orientation.y,
                          msg->pose.orientation.z,
                          msg->pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);

        if (!yaw_init_) {
            init_yaw_ = yaw_;
            last_cmd_yaw_ = yaw_;
            yaw_init_ = true;
            RCLCPP_INFO(get_logger(), "Initial yaw: %.2f°", init_yaw_ * 180 / M_PI);
        }
    }

    /* -------------- Main loop -------------- */
    void loop()
    {
        if (!yaw_init_ || waypoints_.empty()) return;

        /* arm + offboard after 10 set-points */
        if (setpt_cnt_ == 10) {
            publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
            RCLCPP_INFO(get_logger(), "OFFBOARD enabled");
        }

        publish_offboard_mode();
        handle_waypoints();

        setpt_cnt_++;
    }
    void publish_offboard_mode()
    {
        OffboardControlMode m{};
        m.timestamp = now().nanoseconds() / 1e3;
        m.position  = true;
        offboard_ctrl_pub_->publish(m);
    }

    /* -------------- Waypoint engine -------------- */
    void handle_waypoints()
    {
        TrajectorySetpoint sp{};
        sp.timestamp = now().nanoseconds() / 1e3;

        if (wp_idx_ < waypoints_.size()) {

            auto wp  = waypoints_[wp_idx_];
            auto pos = body_to_world(wp[0], wp[1], wp[2]);
            double cmd_yaw = init_yaw_ + wp[3];

            sp.position = { pos[0], pos[1], pos[2] };
            sp.yaw      = cmd_yaw;

            /* errors */
            float dx = x_ - pos[0];
            float dy = y_ - pos[1];
            float dz = z_ - pos[2];
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double yaw_diff = std::fabs(wrap_pi(cmd_yaw - last_cmd_yaw_));

            if (dist < DIST_THRESHOLD_METERS) {

                bool yaw_changed = (yaw_diff > YAW_DIFF_THRESHOLD);

                if (!yaw_changed) {
                    /* immediate advance */
                    RCLCPP_INFO(get_logger(), "WP %zu reached (no yaw)", wp_idx_);
                    wp_idx_++;
                    last_cmd_yaw_ = cmd_yaw;
                    in_dwell_ = false;
                } else {
                    /* need 5-s dwell at low speed */
                    if (speed_ < SPEED_THRESHOLD_M_S) {
                        if (!in_dwell_) { dwell_start_ = now(); in_dwell_ = true; }
                        if ((now() - dwell_start_).seconds() > DWELL_TIME_SEC) {
                            RCLCPP_INFO(get_logger(), "WP %zu complete after yaw dwell", wp_idx_);
                            wp_idx_++;
                            last_cmd_yaw_ = cmd_yaw;
                            in_dwell_ = false;
                        }
                    } else {
                        in_dwell_ = false;  // moving again → reset timer
                    }
                }
            } else {
                in_dwell_ = false;          // outside radius → reset timer
            }

            traj_sp_pub_->publish(sp);
            return;
        }

        /* -------- landing phase --------- */
        if (!landed_) {
            auto last = waypoints_.back();
            auto p = body_to_world(last[0], last[1], 0.0f);
            sp.position = { p[0], p[1], 0.0f };
            sp.yaw = init_yaw_ + last[3];
            traj_sp_pub_->publish(sp);

            publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            landed_ = true;
            RCLCPP_INFO(get_logger(), "LAND command sent");
        }

        traj_sp_pub_->publish(sp);
    }

    /* -------------- CSV loader -------------- */
    void load_csv_waypoints(const std::string& path)
    {
        std::ifstream f(path);
        std::string line;
        bool header = true;
        if (!f.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open CSV %s", path.c_str());
            return;
        }
        while (std::getline(f, line)) {
            if (header) { header = false; continue; }
            std::stringstream ss(line); std::string tok; std::vector<float> v;
            while (std::getline(ss, tok, ',')) v.push_back(std::stof(tok));
            if (v.size() == 4) waypoints_.push_back({ v[0], v[1], v[2], v[3] });
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());
    }
};

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);  // unbuffer stdout
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
