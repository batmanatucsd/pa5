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

#define THRESH_MAG 3

#define LABEL_FOFA "FOFA"
#define LABEL_MOG2 "MOG2"


// Forward Declaration
static void drawMotionIntensity(const cv::Mat&, cv::Mat&);
static void drawOptFlowMap(const cv::Mat&, cv::Mat&,
        int, double, const cv::Scalar&);
static void drawRectFromContours(cv::Mat&,
        std::vector<std::vector<cv::Point> >&);

class MotionDetector
{
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;
    std::vector<std::vector<cv::Point> > contours;

    // FOFA related variables
    int algorithm_mode;
    cv::Mat prev;
    cv::Mat flow;
    cv::Mat uflow;
    cv::Mat cflow;
    cv::Mat testFlow;

    // MOG2 related files
    cv::BackgroundSubtractorMOG2 bsmog;
    cv::Mat fg_mask;

    void callback_image(const sensor_msgs::ImageConstPtr& msg);

    public:
    MotionDetector(ros::NodeHandle n): it(n)
    {
        image_sub = it.subscribe("/my_camera/image_raw", 1, &MotionDetector::callback_image, this);
        image_pub = it.advertise("/my_camera/image_motion_boxed", 1);

        algorithm_mode = FOFA; //intializes the algorithm to FOFA
    }

    bool switch_callback(assignment_5::model_msg::Request &req,
            assignment_5::model_msg::Response &res)
    {
        switch (algorithm_mode = req.model)
        {
            case FOFA: ROS_INFO("requested to use FOFA"); break;
            case MOG2: ROS_INFO("requested to use MOG2"); break;
            default: break;
        }
        return true;
    }
};

void MotionDetector::callback_image(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr frame_original_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(3,3), cv::Point(-1,-1));
    std::string label;

    try {
        frame_original_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //convert to grayscale and make a copy
    cv::Mat frame_gray = frame_original_ptr->image;
    cv::cvtColor(frame_gray, frame_gray, CV_RGB2GRAY);

    //blur the image before applying algorithm
    cv::GaussianBlur(frame_gray, frame_gray, cv::Size(7, 7), 7, cv::BORDER_DEFAULT);
    switch (algorithm_mode) {
        case FOFA:
            if (!prev.empty()){
                cv::calcOpticalFlowFarneback(prev, frame_gray, uflow, 0.5, 2, 3, 2, 5, 1.1, 0);
                cv::cvtColor(prev, cflow, cv::COLOR_GRAY2BGR);
                uflow.copyTo(flow);
                //draws arrows over the prev grayscale frames (cflow)
                drawOptFlowMap(flow, cflow, 16, 1.5, cv::Scalar(0, 255, 0));
                //imshow(FLOW_WINDOW_ARROWS, cflow);
                cv::Mat intensityMap = cv::Mat::zeros(flow.rows, flow.cols, CV_8U);
                //populates the intensityMap with the binary image for flow
                drawMotionIntensity(flow, intensityMap);
                //remove white specks (noise)
                cv::erode(intensityMap, intensityMap, kernel, cv::Point(-1,-1), 7);
                //dilate what's left to help find blob of body
                cv::dilate(intensityMap, intensityMap, kernel, cv::Point(-1,-1), 30);
                cv::findContours(intensityMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                //draw rectanle on the original image
                drawRectFromContours(frame_original_ptr->image, contours);
                //drawContours(intensityMap, contours, -1, cv::Scalar(0,0,255), 3);
                //rectangle(intensityMap, Point(5, 5), Point(100,100), Scalar(255,255,255), 5);
                //imshow(FLOW_WINDOW_BINARY, intensityMap);
            }
            label.clear();
            label.append(LABEL_FOFA);
            break;
        case MOG2:
            bsmog(frame_gray, fg_mask, -1);
            bsmog.set("nmixtures", 3);
            cv::erode(fg_mask, fg_mask, kernel, cv::Point(-1,-1), 10);
            cv::dilate(fg_mask, fg_mask, kernel, cv::Point(-1,-1), 32);
            cv::findContours(fg_mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
            std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
            std::vector<cv::Rect> boundRect( contours.size() );
            for( int i = 0; i < contours.size(); i++ ) {
                cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
                boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
                cv::rectangle(frame_original_ptr->image, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 2, 8, 0 );
            }
            //cv::imshow(MOG2_WINDOW, fg_mask);
            label.clear();
            label.append(LABEL_MOG2);
            break;
    }
    cv::waitKey(1);

    // Add algorithm label to the image
    rectangle(frame_original_ptr->image, cv::Point(10, 10), cv::Point(150,50),
            cv::Scalar(255,255,255), -1);
    putText(frame_original_ptr->image, label, cv::Point(25, 45),
            cv::FONT_HERSHEY_SIMPLEX, 1.5 , cv::Scalar(0,0,0));

    prev = frame_gray.clone();
    image_pub.publish(frame_original_ptr->toImageMsg());

    // Reset Contours
    contours.clear();
}

static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
        double, const cv::Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step) {
        for(int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                    color);
            cv::circle(cflowmap, cv::Point(x,y), 2, color, -1);
        }
    }
}

static void drawMotionIntensity(const cv::Mat& flow, cv::Mat& A)
{
    for(int i = 0; i < flow.rows; i++) {
        for(int j = 0; j < flow.cols; j++) {
            const cv::Point2f& v = flow.at<cv::Point2f>(i, j);
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

static void drawRectFromContours(cv::Mat& frame, std::vector<std::vector<cv::Point> > &contours)
{
    if (contours.empty()) { return; }
    //only draw rectangle on first contour
    std::vector<cv::Point> contour = contours.front();
    int maxSize = 0;

    //find the largest contour
    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin() ; it != contours.end(); ++it){
        if (it->size() > maxSize){
            contour = *it;
            maxSize = it->size();
        }
    }

    int minX = INT_MAX;
    int maxX = 0;
    int minY = INT_MAX;
    int maxY = 0;

    cv::Point point;
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

    cv::rectangle(frame, cv::Point(minX, minY), cv::Point(maxX,maxY), cv::Scalar(0,0,255), 5);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detector");
    ros::NodeHandle nh;

    MotionDetector motion_detector(nh);

    ros::ServiceServer switch_service = nh.advertiseService("model_switch",
            &MotionDetector::switch_callback, &motion_detector);
    ros::spin();

    return 0;
}
