#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "rclcpp/qos.hpp"

class Failure : public rclcpp::Node {
public:
    Failure() : Node("motor_failure_detection"), roll_rate_(0.0), pitch_rate_(0.0), previous_roll_(0.0), previous_pitch_(0.0), i_(0) {
        
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).best_effort();
        
        // Subscription to the sensor data topic
        subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined",
            qos_profile,
            std::bind(&Failure::sensor_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /fmu/out/sensor_combined");

        // Timer for checking motor failure based on the gyro data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Failure::callback, this)
        );
    }

private:
    // Callback for receiving sensor data
    void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        roll_rate_ = msg->gyro_rad[0];
        pitch_rate_ = msg->gyro_rad[1];

        if (i_ == 0) {
            previous_roll_ = roll_rate_;
            previous_pitch_ = pitch_rate_;
            i_ = 1;
        }
    }

    // Timer callback to detect motor failure based on the error thresholds
    void callback() {
        if (i_ == 1) {
            double error_r = roll_rate_ - previous_roll_;
            double error_p = pitch_rate_ - previous_pitch_;

            previous_roll_ = roll_rate_;
            previous_pitch_ = pitch_rate_;



            if ((error_r > 0) && (std::abs(error_r) > 0.6) && (std::abs(error_p) > 0.3 && (error_p < 0))) {
                std::cout << "roll_rate_error: "<< error_r << std::endl;
                std::cout << "pitch_rate_error: "<< error_p << std::endl;
                std::cout << "motor1 failed" << std::endl;
                i_ = 2;
            } else if ((error_r < 0) && (std::abs(error_r) > 0.6) && (error_p < 0) && (std::abs(error_p) > 0.3)) {
                std::cout << "roll_rate_error: "<< error_r << std::endl;
                std::cout << "pitch_rate_error: "<< error_p << std::endl;
                std::cout << "motor3 failed" << std::endl;
                i_ = 2;
            } else if ((error_r > 0) && (std::abs(error_r) > 0.6) && (error_p > 0) && (std::abs(error_p) > 0.3)) {
                std::cout << "roll_rate_error: "<< error_r << std::endl;
                std::cout << "pitch_rate_error: "<< error_p << std::endl;
                std::cout << "motor4 failed" << std::endl;
                i_ = 2;
            } else if ((error_r < 0) && (std::abs(error_r) > 0.6) && (std::abs(error_p) > 0.3) && (error_p > 0)) {
                std::cout << "roll_rate_error: "<< error_r << std::endl;
                std::cout << "pitch_rate_error: "<< error_p << std::endl;
                std::cout << "motor2 failed" << std::endl;
                i_ = 2;
            }
        }
    }

    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    double roll_rate_;
    double pitch_rate_;
    double previous_roll_;
    double previous_pitch_;
    int i_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Failure>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
