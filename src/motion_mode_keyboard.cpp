#include "ros/ros.h"
#include "assignment_5/model_msg.h"

#include <termios.h>

#define FOFA 91 // Key code
#define MOG2 93 // Key code
#define CTRLC 3

void sig_handler(int sigid) {
    ROS_INFO("SIG HANDLER");
}

int kbhit() {
    struct termios oldttr, newttr;
    struct timeval timeout;
    fd_set read_handles;
    int status;

    // Get current stdin attributes
    tcgetattr(STDIN_FILENO, &oldttr);
    tcgetattr(STDIN_FILENO, &newttr);
    newttr.c_lflag &= ~(ICANON | ECHO | ISIG);
    // Check stdin (fd 0) for activity
    FD_ZERO(&read_handles);
    FD_SET(STDIN_FILENO, &read_handles);
    // Set timeout to 0
    timeout.tv_sec = timeout.tv_usec = 0;
    status = select(STDIN_FILENO + 1, &read_handles, NULL, NULL, &timeout);
    if(status < 0)
    {
        printf("select() failed in kbhit()\n");
        exit(1);
    }
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newttr);
    return status;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_mode_keyboard");
    ros::NodeHandle n;

    bool isReady = false;
    bool isFOFA = true;
    bool keepRunning = true;
    int keycode = 0;
    assignment_5::model_msg srv;

    ros::ServiceClient client = n.serviceClient<assignment_5::model_msg>("model_switch");

    ROS_INFO("Press '[' for FOFA, ']' for MOG2...");


    while (keepRunning) {
        if(kbhit()) { keycode = std::getchar(); }
        //ROS_INFO("KEY PRESSED: %d", keycode);
        switch (keycode) {
            case FOFA:
                if (!isFOFA) {
                    srv.request.model = FOFA;
                    isReady = true;
                    isFOFA = true;
                    ROS_INFO("Requested FOFA!");
                }
                break;
            case MOG2:
                if (isFOFA) {
                    srv.request.model = MOG2;
                    isReady = true;
                    isFOFA = false;
                    ROS_INFO("Requested MOG2!");
                }
                break;
            case CTRLC: keepRunning = false; break;
            default:
                        if (!ros::master::check()) { 
                            keepRunning = false;
                        }
                        break;
        }

        if(isReady) {
            if (client.call(srv)) {
                ROS_INFO("Change success!");
            } else {
                ROS_INFO("Change failed!");
            }
            isReady = false;
            // print out successful response
        } else {
            // print out unsuccessful response
        }
        if (!ros::master::check()) { 
            keepRunning = false;
        }
    }

    return 0;
}
