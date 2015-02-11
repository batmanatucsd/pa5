#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "assignment_5/model_msg.h"
//#include "opencv2/imgproc/imgproc.hpp"

#define FOFA 75 // Key code
#define MOG2 77 // Key code


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_mode_keyboard");
  ros::NodeHandle n;

  bool isFOFA = true;
  int keycode = 0;
  std_msgs::String model;
  assignment_5::model_msg srv;

  ros::ServiceClient client = n.serviceClient<assignment_5::model_msg>("motion_switch");

  ROS_INFO("press left arrow key or the right arrow key ");

  while (1) {
    keycode = cv::waitKey(10);
    ROS_INFO("KEY PRESSED: %d", keycode);
    switch (cv::waitKey(0)) {
      case FOFA: 
        model.data = "FOFA";
        srv.request.model = FOFA;
        break;

      case MOG2: 
        model.data = "MOG2";
        srv.request.model = MOG2;
        break;

      default:
        break;
    }

    if(client.call(srv)) {
      // print out successful response
    } else {
      // print out unsuccessful response
    }

    //ROS_INFO("Current Setting: %s", model.data);
  }

  return 0;
}
