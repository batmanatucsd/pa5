#include "ros/ros.h"
#include "assignment_5/model_msg.h"

#define FOFA 91 // Key code
#define MOG2 93 // Key code

bool callBack(assignment_5::model_msg::Request &req,
    assignment_5::model_msg::Response &res) {
  switch (req.model) {
    case FOFA: ROS_INFO("requested to use FOFA"); break;
    case MOG2: ROS_INFO("requested to use MOG2"); break;
    default: break;
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "motion_detector");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("model_switch", callBack);
  ros::spin();

  return 0;
}
