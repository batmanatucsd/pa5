#include "ros/ros.h"
#include "assignment_5/model_msg.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define CROP_WIDTH 320
#define CROP_HEIGHT 240

#define FOFA 91 // Key code
#define MOG2 93 // Key code

#define FLOW_WINDOW "flow"

using namespace cv;

static void drawMotionIntensity(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color);

class MotionDetector
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;

    int algorithm_mode;
    cv::Mat prev;
    cv::Mat flow;
    cv::Mat uflow;
    cv::Mat cflow;

    cv::Mat testFlow;


    void callback_crop(const sensor_msgs::ImageConstPtr& msg);

  public:
    MotionDetector() : it(nh)
    {
      image_sub = it.subscribe("/camera/image_raw", 1, &MotionDetector::callback_crop, this);
      image_pub = it.advertise("/camera/image_raw_cropped", 1);

      cv::namedWindow(FLOW_WINDOW, cv::WINDOW_AUTOSIZE);
      algorithm_mode = FOFA; //intializes the algorithm to FOFA

    }

    bool switch_callback(assignment_5::model_msg::Request &req,
        assignment_5::model_msg::Response &res)
    {
      switch (algorithm_mode = req.model)
      {
        case FOFA: ROS_INFO("requested to use FOFA"); 
            break;
        case MOG2: ROS_INFO("requested to use MOG2"); 
            break;
        default: break;
      }
    }
};

void MotionDetector::callback_crop(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {   //copy the data //TODO maybe change back to copy
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  switch (algorithm_mode)
  {
    case FOFA: 
        if (!prev.empty()){
            calcOpticalFlowFarneback(prev, cv_ptr->image, uflow, 0.5, 4, 3, 5, 5, 1.1, 0);

            cvtColor(prev, cflow, COLOR_GRAY2BGR);
            uflow.copyTo(flow);
            drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
            
            cv::waitKey(1);

            imshow(FLOW_WINDOW, cflow);
        }
        break;
    
    
    case MOG2:
        break;
    
  }

/*
  if (doCrop)
  {
        //create the region to crop
        cv::Rect roi((cv_ptr->image.cols)/2 - CROP_WIDTH/2, (cv_ptr->image.rows)/2 - CROP_HEIGHT/2, CROP_WIDTH, CROP_HEIGHT);

        //rewrite the data with the cropped data
        cv_ptr->image = cv_ptr->image(roi);
  }
*/

  prev = cv_ptr->image.clone();
  image_pub.publish(cv_ptr->toImageMsg());
}

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

static void drawMotionIntensity(const Mat& flow, Mat& intensityMap, int step,
                    double, const Scalar& color)
{
    for(int i = 0; i < flow.rows; i++)
        for(int j = 0; j < flow.cols; j++)
        {
            const Point2f& pointVector = flow.at<Point2f>(i, j);

            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_detector");
  ros::NodeHandle n;

  MotionDetector motion_detector;

  ros::ServiceServer switch_service = n.advertiseService("model_switch", &MotionDetector::switch_callback, &motion_detector);
  ros::spin();


  return 0;
}
