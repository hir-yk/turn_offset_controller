#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>


class TurnOffsetController : public rclcpp::Node {
public:
    TurnOffsetController() : Node("turn_report_subscriber"), in_intersection_(false) {
        subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
                "/vehicle/status/turn_indicators_status", 10,
                std::bind(&TurnOffsetController::listener_callback, this, std::placeholders::_1));

        steering_status_subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
                "/vehicle/status/steering_status", 10,
                std::bind(&TurnOffsetController::steering_status_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<tier4_planning_msgs::msg::LateralOffset>(
                "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset", 10);

        prev_state_ = 1;
        in_intersection_ = false;
    }

private:
    void listener_callback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "I heard: %d", msg->report);

        auto offset_msg = std::make_shared<tier4_planning_msgs::msg::LateralOffset>();

        if (msg->report == prev_state_) {
            return;
        }

        if (msg->report == 1 || in_intersection_) {
            offset_msg->lateral_offset = 0.0;
        } else if (msg->report == 2) {
            offset_msg->lateral_offset = 0.3;
        } else if (msg->report == 3) {
            offset_msg->lateral_offset = -0.3;
        }

        prev_state_ = msg->report;

        publisher_->publish(*offset_msg);
        RCLCPP_INFO(this->get_logger(), "turn_signal_report: %d", msg->report);
        RCLCPP_INFO(this->get_logger(), "publishing_offset: %f", offset_msg->lateral_offset);
    }


void steering_status_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg_ptr) {
    const auto& msg = *msg_ptr; // Convert to a reference
    RCLCPP_DEBUG(this->get_logger(), "Steering tire angle: %f", msg.steering_tire_angle);

    bool new_intersection_status = std::abs(msg.steering_tire_angle) >= 0.00872664;  // 0.5[deg]
    if (new_intersection_status != in_intersection_) {
        in_intersection_ = new_intersection_status;
        if (in_intersection_) {
            auto offset_msg = std::make_shared<tier4_planning_msgs::msg::LateralOffset>();
            offset_msg->lateral_offset = 0.0;
            publisher_->publish(*offset_msg);
            RCLCPP_INFO(this->get_logger(), "In intersection, setting offset to 0.0");
        }
    }
}

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr subscription_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_subscription_;
    rclcpp::Publisher<tier4_planning_msgs::msg::LateralOffset>::SharedPtr publisher_;
    int prev_state_;
    bool in_intersection_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_subscriber = std::make_shared<TurnOffsetController>();

    rclcpp::spin(my_subscriber);
    rclcpp::shutdown();
    return 0;
}
