#include <math.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class Safety : public rclcpp::Node {
    // The class that handles emergency braking

   public:
    Safety() : Node("safety_node") {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1,
            std::bind(&Safety::odomCB, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            std::bind(&Safety::scanCB, this, std::placeholders::_1));

        // Publishers
        drive_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 1);
        ttc_pub_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>("/ttc", 1);

        this->declare_parameter("fov", 50);
        fov_ = this->get_parameter("fov").as_int();
        fov_ *= M_PI / 180.;

        this->declare_parameter("ttc_thresh", -1.2);
        ttc_threshold_ = this->get_parameter("ttc_thresh").as_double();
    }

   private:
    double current_speed_ = 0.0;
    double current_heading_ = 0.0;
    double fov_;
    double ttc_threshold_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ttc_pub_;

    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
        current_heading_ = msg->twist.twist.angular.z;
    }

    void scanCB(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        //! Find angular span of scan
        auto min_angle = scan_msg->angle_min;
        auto max_angle = scan_msg->angle_max;
        auto increment = scan_msg->angle_increment;

        auto num_lasers = static_cast<int>((max_angle - min_angle) / increment);
        auto midpoint = std::round(num_lasers / 2);

        //! Define crop scan zone
        auto half_fov = fov_ / 2;
        auto half_fov_idx = static_cast<int>(half_fov / increment);

        // Because even num_lasers
        auto crop_start = midpoint - 1 - half_fov_idx;
        auto min_crop_angle = -half_fov;
        auto crop_end = midpoint + half_fov_idx;

        //! Assign scans/angles in FOV
        std::vector<double> scan_angles;
        std::vector<double> scans;
        int j = 1;
        for (std::size_t i = crop_start; i <= crop_end; ++i) {
            scan_angles.push_back(min_crop_angle + j * increment);
            scans.push_back(scan_msg->ranges[i]);
            j++;
        }

        //! Filter nans and infs
        for (std::size_t i = 0; i < scans.size(); ++i) {
            if (!std::isfinite(scans[i])) {
                scans.erase(scans.begin() + i);
                scan_angles.erase(scan_angles.begin() + i);
            }
        }

        //! Calculate TTC
        /**
         * Positive r_dot: Scan range measurement is expanding (good)
         * Negative r_dot: Scan range measurement is shrinking (bad)
         */
        std::vector<double> r_dot;
        for (auto &angle : scan_angles) {
            r_dot.push_back(current_speed_ * std::cos(angle));
        }

        // Evaluate TTC
        std_msgs::msg::Float32MultiArray ttc_msg;
        std::vector<double> ttcs;
        bool emergency_brake = false;
        for (std::size_t i = 0; i < scans.size(); ++i) {
            double ttc = scans[i] / -r_dot[i];
            ttcs.push_back(ttc);
            ttc_msg.data.push_back(ttc);
            if (ttc > ttc_threshold_ && ttc < 0.) {
                emergency_brake = true;
                break;
            }
        }

        if (emergency_brake) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE ENGAGED");
            ackermann_msgs::msg::AckermannDriveStamped msg;
            msg.drive.speed = 0.;
            msg.drive.acceleration = 0.;
            msg.drive.jerk = 0.;
            drive_pub_->publish(msg);
        }
        ttc_pub_->publish(ttc_msg);
    }
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}