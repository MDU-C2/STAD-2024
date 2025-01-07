#include <functional>
#include <memory>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "serial.hpp"
#include "winch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define USB_TARGET "/dev/ttyACM0"
#define TOPIC_NAME "topic"
#define TAKEOFF_MESSAGE "TO"
#define COORDS_FORMAT "[%f %f %f]"

#define SLACK_BUFFER_MM 300

struct Coordinate
{
    float x;
    float y;
    float z;

    float euclidean_distance(const struct Coordinate other) const
    {
        return sqrt(pow(x - other.x, 2) \
            + pow(y - other.y, 2) \
            + pow(z - other.z, 2) \
        );
    }
};
typedef struct Coordinate Coordinate;

// We consider the drone to start at the origin
Coordinate last_position = Coordinate{0, 0, 0};
Coordinate ORIGIN = Coordinate{0, 0, 0};

// set to true is you want to disable the winch
// from giving slack at the beginning
bool has_taken_off = false;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            TOPIC_NAME,
            10, // QoS
            std::bind(&MinimalSubscriber::topic_callback, this, _1)
        );
        
        serial_fd_ = openSerialPort(USB_TARGET);

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
        else
        {
            configureSerialPort(serial_fd_, B1000000);
        }
    }

    ~MinimalSubscriber()
    {
        if (serial_fd_ < 0)
        {
            return;
        }

        closeSerialPort(serial_fd_);
    }

private:
    void topic_callback(const std_msgs::msg::String &msg) const
    {
        // This is responsible for giving a certain amount of slack to the system
        // so any unexpected movement doesn't cause the winch to prevent the drone from moving.
        if (!has_taken_off) {
            if (msg.data == TAKEOFF_MESSAGE)
            {
                send_winch_command(CW, SLACK_BUFFER_MM, serial_fd_);
                has_taken_off = true;
            }

            return;
        }

        Coordinate new_position = parse_coordinates(msg.data);

        float last_distance_to_origin = last_position.euclidean_distance(ORIGIN);
        float new_distance_to_origin = new_position.euclidean_distance(ORIGIN);

        // TODO: configure the direction
        uint8_t direction = new_distance_to_origin > last_distance_to_origin ? CW : CCW;
        
        float euclidean_distance = last_position.euclidean_distance(new_position); // in meters
        euclidean_distance *= 1000; // to mm

        send_winch_command(direction, euclidean_distance, serial_fd_);

        last_position = new_position;
    }

    Coordinate parse_coordinates(const std::string &data) const
    {
        Coordinate coord;
        sscanf(data.c_str(), COORDS_FORMAT, &coord.x, &coord.y, &coord.z);
        return coord;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int serial_fd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}