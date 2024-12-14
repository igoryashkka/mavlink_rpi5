#include <iostream>
#include "mavlink/common/mavlink.h"
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <stdexcept>

#define SERIAL_PORT "/dev/ttyAMA0"
#define BAUD_RATE B57600


int configure_serial_port(const char* port, speed_t baud_rate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw std::runtime_error("Failed to open serial port");
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Failed to get terminal attributes");
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                    // disable break processing
    tty.c_lflag = 0;                           // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                           // no remapping, no delays
    tty.c_cc[VMIN] = 1;                        // read doesn't block
    tty.c_cc[VTIME] = 1;                       // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);    // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);           // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);         // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Failed to set terminal attributes");
    }

    return fd;
}

void request_raw_imu(int serial_fd, int system_id, int component_id) {
    mavlink_message_t message;
    mavlink_command_long_t command = {};
    command.target_system = system_id;
    command.target_component = component_id;
    command.command = MAV_CMD_REQUEST_MESSAGE;
    command.confirmation = 0;
    command.param1 = MAVLINK_MSG_ID_RAW_IMU;
    command.param2 = 0;
    command.param3 = 0;
    command.param4 = 0;
    command.param5 = 0;
    command.param6 = 0;
    command.param7 = 0;

    mavlink_msg_command_long_encode(system_id, component_id, &message, &command);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    size_t len = mavlink_msg_to_send_buffer(buffer, &message);
    write(serial_fd, buffer, len);
}

// Function to decode and display RAW_IMU data
void handle_raw_imu(const mavlink_message_t& message) {
    mavlink_raw_imu_t raw_imu;
    mavlink_msg_raw_imu_decode(&message, &raw_imu);

    std::cout << "Raw IMU Data:" << std::endl;
    std::cout << " Accel X: " << raw_imu.xacc << std::endl;
    std::cout << " Accel Y: " << raw_imu.yacc << std::endl;
    std::cout << " Accel Z: " << raw_imu.zacc << std::endl;
    std::cout << " Gyro X: " << raw_imu.xgyro << std::endl;
    std::cout << " Gyro Y: " << raw_imu.ygyro << std::endl;
    std::cout << " Gyro Z: " << raw_imu.zgyro << std::endl;
}

int main() {
    try {
        std::cout << "Connecting to MAVLink via serial..." << std::endl;
        int serial_fd = configure_serial_port(SERIAL_PORT, BAUD_RATE);

        int system_id = 1;
        int component_id = 1;

        request_raw_imu(serial_fd, system_id, component_id);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t message;
        mavlink_status_t status;

        while (true) {
            int n = read(serial_fd, buffer, sizeof(buffer));
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
                        if (message.msgid == MAVLINK_MSG_ID_RAW_IMU) {
                            handle_raw_imu(message);
                        }
                    }
                }
            }
            usleep(100000); 
        }

        close(serial_fd);

    } catch (const std::exception& ex) {
        std::cerr << "An error occurred: " << ex.what() << std::endl;
    }

    return 0;
}

