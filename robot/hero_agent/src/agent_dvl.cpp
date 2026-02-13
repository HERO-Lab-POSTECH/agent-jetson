#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include "hero_msgs/hero_agent_dvl_velocity.h"

#include <cstdio>
#include <cstring>
#include <string>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

static int fd;

void msgCallback_command(const std_msgs::Int8::ConstPtr &msg)
{
    int8_t command = msg->data;

    if (command == 0) {
        unsigned char com[] = {'w', 'c', 'v', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcv");
    } else if (command == 1) {
        unsigned char com[] = {'w', 'c', 'r', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcr");
    } else if (command == 2) {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', 'y', ',', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcs,,,y,");
    } else if (command == 3) {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', 'n', ',', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcs,,,n,");
    } else if (command == 4) {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', ',', 'y', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcs,,,,y");
    } else if (command == 5) {
        unsigned char com[] = {'w', 'c', 's', ',', ',', ',', ',', 'n', '\r', '\n'};
        write(fd, com, sizeof(com));
        ROS_DEBUG("DVL cmd: wcs,,,,n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_dvl");
    ros::NodeHandle nh("~");

    // Load serial port parameters from YAML (defaults = original hardcoded values)
    std::string serial_port;
    int baudrate_val;
    int loop_rate_hz;
    double log_period;

    nh.param<std::string>("serial/port", serial_port, "/dev/ttyUSB1");
    nh.param<int>("serial/baudrate", baudrate_val, 115200);
    nh.param<int>("loop_rate_hz", loop_rate_hz, 100);
    nh.param<double>("log_period", log_period, 1.0);

    // Map baudrate integer to termios constant
    speed_t baudrate;
    switch (baudrate_val) {
        case 9600:   baudrate = B9600;   break;
        case 19200:  baudrate = B19200;  break;
        case 38400:  baudrate = B38400;  break;
        case 57600:  baudrate = B57600;  break;
        case 115200: baudrate = B115200; break;
        default:     baudrate = B115200;
            ROS_WARN("Unknown baudrate %d, defaulting to 115200", baudrate_val);
            break;
    }

    int res;
    struct termios oldtio, newtio;
    char buf[512];

    double time_stamp, dvl_x, dvl_y, dvl_z, pos_std, roll, pitch, yaw;
    double vx, vy, vz, altitude, fom, v_time;
    char valid;
    double covariance[9];
    int tov, tot;
    int status;

    fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        ROS_ERROR("Failed to open serial port: %s", serial_port.c_str());
        return -1;
    }

    tcgetattr(fd, &oldtio);
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    newtio.c_cc[VEOF]  = 4;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN]  = 1;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    ros::Rate loop_rate(loop_rate_hz);

    ros::Subscriber sub_command =
        nh.subscribe("/hero_agent/dvl_command", 100, msgCallback_command);

    ros::Publisher pub_dvl_velocity =
        nh.advertise<hero_msgs::hero_agent_dvl_velocity>("/hero_agent/dvl_velocity", 100);

    hero_msgs::hero_agent_dvl_velocity dvl_velocity_msg;

    // Startup banner
    ROS_INFO("===================================");
    ROS_INFO("  Agent DVL Node Initialized");
    ROS_INFO("  Port: %s  Baud: %d", serial_port.c_str(), baudrate_val);
    ROS_INFO("  Loop rate: %d Hz", loop_rate_hz);
    ROS_INFO("===================================");

    while (ros::ok())
    {
        // Non-blocking read with 100ms timeout (allows ros::ok() check)
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms

        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0) {
            // Timeout or error — just loop back to check ros::ok()
            ros::spinOnce();
            continue;
        }

        res = read(fd, buf, 511);
        if (res <= 0) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        buf[res] = 0;

        if (buf[0] == 'w' && buf[1] == 'r' && buf[2] == 'p') {
            sscanf(buf, "wrp,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d*de",
                   &time_stamp, &dvl_x, &dvl_y, &dvl_z, &pos_std,
                   &roll, &pitch, &yaw, &status);
        }
        else if (buf[0] == 'w' && buf[1] == 'r' && buf[2] == 'z') {
            sscanf(buf, "wrz,%lf,%lf,%lf,%c,%lf,%lf,%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf,%d,%d,%lf,%d*de",
                   &vx, &vy, &vz, &valid, &altitude, &fom,
                   &covariance[0], &covariance[1], &covariance[2],
                   &covariance[3], &covariance[4], &covariance[5],
                   &covariance[6], &covariance[7], &covariance[8],
                   &tov, &tot, &v_time, &status);

            dvl_velocity_msg.VX = vx;
            dvl_velocity_msg.VY = vy;
            dvl_velocity_msg.TIME = v_time;
            dvl_velocity_msg.VALID = valid;

            pub_dvl_velocity.publish(dvl_velocity_msg);

            ROS_INFO_THROTTLE(log_period, "DVL vel: vx=%.3f vy=%.3f valid=%c", vx, vy, valid);
        }
        else if (buf[0] == 'w' && buf[1] == 'r' &&
                 (buf[2] == 'a' || buf[2] == 'n' || buf[2] == 'v')) {
            ROS_DEBUG("DVL msg: %s", buf);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    return 0;
}
