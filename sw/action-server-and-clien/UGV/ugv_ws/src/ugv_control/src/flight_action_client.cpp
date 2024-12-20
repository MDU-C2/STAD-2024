/* maked by Mohamad Hamdi Alhaji Shommo*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uav_control_interfaces/action/flight.hpp"

using Flight = uav_control_interfaces::action::Flight;
using GoalHandleFlight = rclcpp_action::ClientGoalHandle<Flight>;

class FlightActionClient : public rclcpp::Node {
public:
    FlightActionClient() : Node("flight_action_client") {
        this->client_ptr_ = rclcpp_action::create_client<Flight>(
            this, "flight");

        RCLCPP_INFO(this->get_logger(), "Press '1' to send flight goal or '0' to land.");
    }

    void send_goal(bool fly) {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = Flight::Goal();
        if (fly) {
            goal_msg.altitude = 10.0f; // Example altitude
            RCLCPP_INFO(this->get_logger(), "Sending goal to fly to altitude: %.2f", goal_msg.altitude);
        } else {
            goal_msg.altitude = 0.0f; // Example altitude for landing
            RCLCPP_INFO(this->get_logger(), "Sending goal to land.");
        }

        auto send_goal_options = rclcpp_action::Client<Flight>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FlightActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&FlightActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&FlightActionClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Flight>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_future<GoalHandleFlight::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void feedback_callback(GoalHandleFlight::SharedPtr,
                           const std::shared_ptr<const Flight::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());
    }

    void result_callback(const GoalHandleFlight::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded: Final altitude %.2f", result.result->current_altitude);
                RCLCPP_INFO(this->get_logger(), "UAV is now in waiting status.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightActionClient>();

    char input;
    while (rclcpp::ok()) {
        std::cout << "Enter command (1: Fly, 0: Land, q: Quit): ";
        std::cin >> input;
        if (input == '1') {
            node->send_goal(true); // Fly command
        } else if (input == '0') {
            node->send_goal(false); // Land command
        } else if (input == 'q') {
            break; // Quit
        } else {
            std::cout << "Invalid input. Try again.\n";
        }

        // Spin the node to handle feedback and result updates
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
