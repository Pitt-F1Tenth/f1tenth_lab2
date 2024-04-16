#include <math.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

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
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 1,
            std::bind(&Safety::driveCB, this, std::placeholders::_1));
        teleop_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/teleop", 1,
            std::bind(&Safety::teleopCB, this, std::placeholders::_1));
        autonomy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_autonomous", 1,
            std::bind(&Safety::autonomyCB, this, std::placeholders::_1));

        // Publishers
        safe_drive_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/safe_drive", 1);
        ttc_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/ttc", 1);

        this->declare_parameter("fov", 30);
        fov_ = this->get_parameter("fov").as_int();
        fov_ *= M_PI / 180.;

        this->declare_parameter("ttc_thresh", -0.8);
        ttc_threshold_ = this->get_parameter("ttc_thresh").as_double();

        this->declare_parameter("brake", false);
        emergency_brake_override_ = this->get_parameter("brake").as_bool();

	// Timer
	param_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
			std::bind(&Safety::Timer, this));
    }

    void Timer() {
        fov_ = this->get_parameter("fov").as_int();
	fov_ *= M_PI / 180.;
        ttc_threshold_ = this->get_parameter("ttc_thresh").as_double();
        emergency_brake_override_ = this->get_parameter("brake").as_bool();
    }	

   private:
    double current_speed_ = 0.0;
    double current_heading_ = 0.0;
    double fov_;
    double ttc_threshold_;
    bool emergency_brake_override_ = false;
    bool autonomy_enabled_ = false;
    ackermann_msgs::msg::AckermannDriveStamped teleop_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped drive_cmd_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        safe_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ttc_pub_;
    rclcpp::TimerBase::SharedPtr param_timer_;

    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
        current_heading_ = msg->twist.twist.angular.z;
    }

    void teleopCB(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        teleop_cmd_ = *msg;
        current_speed_ = msg->drive.speed;
    }

    void driveCB(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        drive_cmd_ = *msg;
    }

    void autonomyCB(const std_msgs::msg::Bool::ConstSharedPtr msg) {
        autonomy_enabled_ = msg->data;
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
        std_msgs::msg::Float32 ttc_msg;
        ttc_msg.data = -1;
        bool emergency_brake = false;
        for (std::size_t i = 0; i < scans.size(); ++i) {
            double ttc = scans[i] / -r_dot[i];
            // ttc_msg.data.push_back(ttc);
            if (ttc > ttc_threshold_ && ttc < 0.) {
                ttc_msg.data = ttc;
                emergency_brake = true;
                break;
            }
        }

        if (emergency_brake_override_ || emergency_brake) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY BRAKE ENGAGED");
            ackermann_msgs::msg::AckermannDriveStamped msg = teleop_cmd_;
            msg.drive.speed = 0.;
            msg.drive.acceleration = 0.;
            msg.drive.jerk = 0.;
            safe_drive_pub_->publish(msg);
        } else if (autonomy_enabled_) {
            safe_drive_pub_->publish(drive_cmd_);
        } else {
            safe_drive_pub_->publish(teleop_cmd_);
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

