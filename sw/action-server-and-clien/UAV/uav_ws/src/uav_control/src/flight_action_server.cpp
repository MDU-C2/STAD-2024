/* maked by Mohamad Hamdi Alhaji Shommo*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "uav_control_interfaces/action/flight.hpp"
#include <std_msgs/msg/int32.hpp>

using Flight = uav_control_interfaces::action::Flight;
using GoalHandleFlight = rclcpp_action::ServerGoalHandle<Flight>;

class FlightActionServer : public rclcpp::Node {
public:
    FlightActionServer() : Node("flight_action_server"), status_("WAITING") {
        using namespace std::placeholders;
        
        this->action_server_ = rclcpp_action::create_server<Flight>(
            this,
            "flight",
            std::bind(&FlightActionServer::handle_goal, this, _1, _2),
            std::bind(&FlightActionServer::handle_cancel, this, _1),
            std::bind(&FlightActionServer::handle_accepted, this, _1)
        );

        // Subscriber to receive commands from UGV
        command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "flight_command",
            10,
            std::bind(&FlightActionServer::handle_command, this, _1));
    }

private:
    rclcpp_action::Server<Flight>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscriber_;
    std::string status_;
    int current_command_ = -1; // -1: No command, 0: Land, 1: Fly

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Flight::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal to fly to altitude: %.2f", goal->altitude);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFlight> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFlight> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&FlightActionServer::execute, this, _1), goal_handle}.detach();
    }

    void handle_command(const std_msgs::msg::Int32::SharedPtr msg) {
        current_command_ = msg->data;
        if (current_command_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Received command to fly.");
            status_ = "FLYING";
        } else if (current_command_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Received command to land.");
            status_ = "LANDING";
        }
    }

    void execute(const std::shared_ptr<GoalHandleFlight> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Flight::Feedback>();
        auto result = std::make_shared<Flight::Result>();

        float current_altitude = 0.0f;
        rclcpp::Rate loop_rate(1);

        while (rclcpp::ok()) {
            if (current_command_ == 1 && current_altitude < goal->altitude) {
                // Fly to the target altitude
                current_altitude += 1.0f;
                feedback->status = "Ascending...";
                RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude);
            } else if (current_command_ == 0 && current_altitude > 0.0f) {
                // Land the UAV
                current_altitude -= 1.0f;
                feedback->status = "Descending...";
                RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude);
            } else if (current_command_ == 0 && current_altitude <= 0.0f) {
                // UAV has landed
                feedback->status = "WAITING";
                result->current_altitude = current_altitude;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded. UAV is waiting on the ground.");
                status_ = "WAITING";
                break;
            } else if (current_command_ == 1 && current_altitude >= goal->altitude) {
                // Hover at the target altitude
                feedback->status = "Hovering...";
                result->current_altitude = current_altitude;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded. Reached altitude: %.2f", current_altitude);
                break;
            }

            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
