#include "ros/ros.h"
#include "assignment_5/model_msg.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CROP_WIDTH 320
#define CROP_HEIGHT 240

#define FOFA 91 // Key code
#define MOG2 93 // Key code


class MotionDetector
{

public:
    MotionDetector() : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1, &MotionDetector::callback_crop, this);
        image_pub = it.advertise("/camera/image_raw_cropped", 1);

    }
    bool callback(assignment_5::model_msg::Request &req, assignment_5::model_msg::Response &res); 

         
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;


    void callback_crop(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {   //copy the data
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        } 

        //create the region to crop
        cv::Rect roi((cv_ptr->image.cols)/2 - CROP_WIDTH/2, (cv_ptr->image.rows)/2 - CROP_HEIGHT/2, CROP_WIDTH, CROP_HEIGHT);

        //rewrite the data with the cropped data
        cv_ptr->image = cv_ptr->image(roi);

        image_pub.publish(cv_ptr->toImageMsg());
    }

};

bool MotionDetector::callback(assignment_5::model_msg::Request &req,
        assignment_5::model_msg::Response &res) 
{
    switch (req.model) 
    {
        case FOFA: ROS_INFO("requested to use FOFA"); break;
        case MOG2: ROS_INFO("requested to use MOG2"); break;
        default: break;

    }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "motion_detector");
    
    ros::NodeHandle n;
  
  MotionDetector motion_detector;

    ros::ServiceServer switch_service = n.advertiseService("model_switch", &MotionDetector::callBack, 
                                                                                    &motion_detector);
  ros::spin();


    return 0;
}
    
