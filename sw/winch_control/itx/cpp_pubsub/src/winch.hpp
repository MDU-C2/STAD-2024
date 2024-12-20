/*
* winch.hpp
* 
* The middleware to control the winch.
*/

#ifndef WINCH_HPP
#define WINCH_HPP

#include "serial.hpp"
#include "rclcpp/rclcpp.hpp"

// With a radius of 45mm => perimeter = 282.743338823
#define PERIMETER 282.743338823
// configured in the stepper driver
#define STEPS_PER_REV 400.0
float ANGLE_PER_STEP = 360.0 / STEPS_PER_REV;
float DISTANCE_PER_STEP_MM = PERIMETER * ANGLE_PER_STEP / 360.0;

#define STEPPER_GEARS 15
#define WINCH_GEARS 64
float GEAR_RATIO = WINCH_GEARS / STEPPER_GEARS;

// TODO: state which direction give/takes slack
#define CW 0
#define CCW 1

// There is only 15 bits available for the steps
int MAX_STEPS = pow(2, 15)-1;

/**
 * @brief Packs the direction and distance into a 16bit packet
 * 
 *  packet (binary):
 *  x xxx xxxx xxxx xxxx
 *  ^ ^^^^^^^^^^^^^^^^^^
 *  |            |
 *  direction    |
 *               |
 *          steps to move (15bits)
 * 
 * @param direction 0 for CW, 1 for CCW
 * @param distance in mm
 * @return uint16_t 
*/
uint16_t serial_packet_packer(uint8_t direction, float distance) {

    float steps = distance / DISTANCE_PER_STEP_MM;
    steps *= GEAR_RATIO;

    // 15bit overflow protection
    if (steps > MAX_STEPS) { steps = MAX_STEPS; }

    uint16_t int_steps = static_cast<uint16_t>(steps);

    uint8_t first_byte = (int_steps >> 8);  // isolate the first 8 bits
    first_byte &= 0x7F;                     // clear the direction bit
    first_byte |= (direction << 7);         // set the direction bit

    uint8_t second_byte = int_steps & 0xFF;

    uint16_t packet = (first_byte << 8) | second_byte;
    return packet;
}


/**
 * @brief Sends a winch command to the serial port
 * 
 * @param direction 0 for CW, 1 for CCW
 * @param distance in mm
 * @param serial_fd file descriptor of the serial port
*/
void send_winch_command(bool direction, float distance, int serial_fd) {
    if (serial_fd < 0) { return; }

    uint16_t command = serial_packet_packer(direction, distance);

    // this weird casting is needed for the serial API
    char command_bytes[2];
    command_bytes[0] = static_cast<char>(command >> 8);
    command_bytes[1] = static_cast<char>(command & 0xFF);

    writeToSerialPort(serial_fd, command_bytes, sizeof(command_bytes));
}

#endif // WINCH_HPP