#include "ros/ros.h"
#include "std_msgs/String.h"
#include "assignment_5/model_msg.h"
#include <termios.h>

#define FOFA 91 // Key code
#define MOG2 93 // Key code

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_mode_keyboard");
  ros::NodeHandle n;

  bool isReady = false;
  int keycode = 0;
  struct termios termttr;
  assignment_5::model_msg srv;

  ros::ServiceClient client = n.serviceClient<assignment_5::model_msg>("model_switch");

  ROS_INFO("Press '[' for FOFA, ']' for MOG2...");

  tcgetattr(STDIN_FILENO, &termttr);
  termttr.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &termttr);

  while (1) {
    keycode = std::getchar();
    //ROS_INFO("KEY PRESSED: %d", keycode);
    switch (keycode) {
      case FOFA:
        srv.request.model = FOFA;
        isReady = true;
        break;

      case MOG2:
        srv.request.model = MOG2;
        isReady = true;
        break;

      default:
        break;
    }

    if(isReady) {
      client.call(srv);
      isReady = false;
      // print out successful response
    } else {
      // print out unsuccessful response
    }
  }

  tcgetattr(STDIN_FILENO, &termttr);
  termttr.c_lflag &= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &termttr);


  return 0;
}
