#include "ros/ros.h"
#include "assignment_5/model_msg.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>

#include <cmath>
#include <vector>

#define CROP_WIDTH 320
#define CROP_HEIGHT 240

#define FOFA 91 // Key code
#define MOG2 93 // Key code

#define FLOW_WINDOW_ARROWS "Flow Arrows"
#define FLOW_WINDOW_BINARY "Flow Binary"
#define THRESH_MAG 3 //threshold for displaying farneback optical flow as a binary image
#define MOG2_WINDOW "MOG2"

using namespace cv;

static void drawMotionIntensity(const Mat& flow, Mat& A);
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
        double, const Scalar& color);

static void drawRectFromContours(Mat& frame, std::vector<std::vector<cv::Point> > &contours);

class MotionDetector
{
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;

    int algorithm_mode;
    cv::Mat prev;
    cv::Mat flow;
    cv::Mat uflow;
    cv::Mat cflow;
    cv::Mat testFlow;

    // MOG2 related files
    //cv::Ptr<cv::BackgroundSubtractor> bsmog;
    cv::BackgroundSubtractorMOG2 bsmog;
    cv::Mat fg_mask;
    std::vector<std::vector<cv::Point> > contours;

    void callback_crop(const sensor_msgs::ImageConstPtr& msg);

    public:
    MotionDetector(ros::NodeHandle nh) : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1, &MotionDetector::callback_crop, this);
        image_pub = it.advertise("/camera/image_motion_boxed", 1);

        //cv::namedWindow(FLOW_WINDOW, cv::WINDOW_AUTOSIZE);
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
    cv_bridge::CvImagePtr frame_original_ptr;
    try
    {   //copy the data //TODO maybe change back to copy
        frame_original_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //convert to grayscale and make a copy
    Mat frame_gray = frame_original_ptr->image;
    cvtColor(frame_gray, frame_gray, CV_RGB2GRAY);

    switch (algorithm_mode)
    {
        case FOFA:
            if (!prev.empty()){

                //blur the image before applying optical flow
                cv::GaussianBlur(frame_gray, frame_gray, cv::Size(7, 7), 7, BORDER_DEFAULT);

                calcOpticalFlowFarneback(prev, frame_gray, uflow, 0.5, 2, 3, 2, 5, 1.1, 0);

                cvtColor(prev, cflow, COLOR_GRAY2BGR);
                uflow.copyTo(flow);
                //draws arrows over the prev grayscale frames (cflow)
                drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
                imshow(FLOW_WINDOW_ARROWS, cflow);
                cv::Mat intensityMap = Mat::zeros(flow.rows, flow.cols, CV_8U);
                //populates the intensityMap with the binary image for flow
                drawMotionIntensity(flow, intensityMap);
                cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3), Point(-1,-1));
                //remove white specks (noise)
                erode(intensityMap, intensityMap, kernel, Point(-1,-1), 7);
                //dilate what's left to help find blob of body
                dilate(intensityMap, intensityMap, kernel, Point(-1,-1), 30);
                findContours(intensityMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

                //draw rectanle on the original image
                drawRectFromContours(frame_original_ptr->image, contours);

                //drawContours(intensityMap, contours, -1, cv::Scalar(0,0,255), 3);
                //rectangle(intensityMap, Point(5, 5), Point(100,100), Scalar(255,255,255), 5);

                imshow(FLOW_WINDOW_BINARY, intensityMap);

                cv::waitKey(1);

            }
            break;
        case MOG2:
            bsmog(frame_gray, fg_mask, -1);
            bsmog.set("nmixtures", 3);
            bsmog.set("detectShadows", 1);
            cv::erode(fg_mask,fg_mask,cv::Mat());
            cv::dilate(fg_mask,fg_mask,cv::Mat());
            cv::findContours(fg_mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
            cv::drawContours(frame_gray,contours,-1,cv::Scalar(0,0,255),2);
            cv::imshow(MOG2_WINDOW, fg_mask);
            cv::waitKey(1);
            break;
    }
    
    contours.clear();
    prev = frame_gray.clone();
    image_pub.publish(frame_original_ptr->toImageMsg());
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

static void drawMotionIntensity(const Mat& flow, Mat& A)
{
    for(int i = 0; i < flow.rows; i++) {
        for(int j = 0; j < flow.cols; j++)
        {
            const Point2f& v = flow.at<Point2f>(i, j);
            const int mag = sqrt(v.x*v.x + v.y*v.y);
            //line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
            //     color);
            //circle(cflowmap, Point(x,y), 2, color, -1);

            //int x = flow.data[flow.step[0]*i + flow.step[1]*j + 0];
            //int y = flow.data[flow.step[0]*i + flow.step[1]*j + 1];

            //std::cout << "x: " << x << std::endl;
            //std::cout << "y: " << y << std::endl;

            //A.data[A.step[0]*i + A.step[1]*j + 0] = sqrt(x*x + y*y);
            if (mag > THRESH_MAG){
                A.data[A.step[0]*i + A.step[1]*j + 0] = 255;
            }
        }
    }
}

static void drawRectFromContours(Mat& frame, std::vector<std::vector<cv::Point> > &contours)
{
    //only draw rectangle on first contour
    std::vector<Point> contour = contours.front();
    int maxSize = 0;

    //find the largest contour
    for (std::vector<std::vector<Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){
        if (it->size() > maxSize){
            contour = *it;
            maxSize = it->size();
        }
    }

    int minX = INT_MAX;
    int maxX = 0;
    int minY = INT_MAX;
    int maxY = 0;

    Point point;
    for (int i = 0; i < contour.size(); i++){
        point = contour.at(i);

        if (point.x < minX){
            minX = point.x;
        }
        if (point.x > maxX){
            maxX = point.x;
        }
        if (point.y < minY){
            minY = point.y;
        }
        if (point.y > maxY){
            maxY = point.y;
        }
    }

    rectangle(frame, Point(minX, minY), Point(maxX,maxY), Scalar(0,0,255), 5);

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detector");
    ros::NodeHandle n;

    MotionDetector motion_detector(n);

    ros::ServiceServer switch_service = n.advertiseService("model_switch",
            &MotionDetector::switch_callback, &motion_detector);
    ros::spin();

    return 0;
}
