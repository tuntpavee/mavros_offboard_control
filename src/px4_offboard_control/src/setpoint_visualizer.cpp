#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using px4_msgs::msg::TrajectorySetpoint;
using std::placeholders::_1;

class SetpointVisualizer : public rclcpp::Node
{
public:
    SetpointVisualizer() : Node("setpoint_visualizer")
    {
        frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

        pose_sub_ = this->create_subscription<PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&SetpointVisualizer::onPose, this, _1));

        setpoint_sub_ = this->create_subscription<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10,
            std::bind(&SetpointVisualizer::onSetpoint, this, _1));

        pose_path_pub_ = this->create_publisher<Path>("/enu/pose_path", 10);
        setpoint_path_pub_ = this->create_publisher<Path>("/enu/setpoint_path", 10);

        pose_path_.header.frame_id = frame_id_;
        setpoint_path_.header.frame_id = frame_id_;

        RCLCPP_INFO(this->get_logger(), "SetpointVisualizer Path Publisher Started");
    }

private:
    static double wrap_pi(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    void onPose(const PoseStamped::SharedPtr msg)
    {
        pose_path_.header.stamp = this->now();
        pose_path_.poses.push_back(*msg);
        pose_path_pub_->publish(pose_path_);
    }

    void onSetpoint(const TrajectorySetpoint::SharedPtr sp)
    {
        if (!std::isfinite(sp->position[0]) || !std::isfinite(sp->position[1]) ||
            !std::isfinite(sp->position[2]) || !std::isfinite(sp->yaw))
            return;

        PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = frame_id_;

        pose.pose.position.x = sp->position[1];  // E → x
        pose.pose.position.y = sp->position[0];  // N → y
        pose.pose.position.z = -sp->position[2]; // -D → z

        double yaw_enu = wrap_pi(M_PI / 2.0 - sp->yaw);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_enu);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        setpoint_path_.header.stamp = this->now();
        setpoint_path_.poses.push_back(pose);
        setpoint_path_pub_->publish(setpoint_path_);
    }

    std::string frame_id_;
    Path pose_path_;
    Path setpoint_path_;

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Publisher<Path>::SharedPtr pose_path_pub_;
    rclcpp::Publisher<Path>::SharedPtr setpoint_path_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointVisualizer>());
    rclcpp::shutdown();
    return 0;
}