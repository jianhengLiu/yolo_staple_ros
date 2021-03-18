#include "staple_tracker.hpp"

#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros_yolo/yolo.h>

using namespace std;
using namespace cv;

ros::ServiceClient client;

string tracking_label = "xxx";

bool is_tracking = false;
STAPLE_TRACKER staple;

void trackerStapleInit(Mat template_img, cv::Rect init_ROI) {
    namedWindow("STAPLE", cv::WINDOW_AUTOSIZE);

    staple.tracker_staple_initialize(template_img, init_ROI);
    staple.tracker_staple_train(template_img, true);
}

void trackerStaple(cv::Mat input) {
    if (!is_tracking) {
        std::cout << "STAPLE tracker is not initialized!" << std::endl;
        return;
    }

    cv::Rect_<float> location = staple.tracker_staple_update(input);
    staple.tracker_staple_train(input, false);

    if (staple.getMaxPro() < 0.2) {
        is_tracking = false;
    }

    cv::rectangle(input, location, cv::Scalar(0, 128, 255), 2);
                cv::putText(input, tracking_label, location.tl(), cv::FONT_HERSHEY_COMPLEX,
                            1, cv::Scalar(0, 0, 255),
                            1, 0);

    cv::imshow("STAPLE", input);
    cv::waitKey(1);
}

void callbackCamera(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;

    if (!is_tracking) {
        ros_yolo::yolo srv;
        srv.request.image = *img_msg;
        if (client.call(srv)) {
            ROS_INFO("request succeed");
            cout << srv.response.results.size() << endl;
            for (auto &result:srv.response.results) {
                auto xyxy = result.bbox.xyxy;
                cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]);

                if (result.label == tracking_label) {
                    cv::Rect init_ROI = cv::Rect(p1, p2);
                    trackerStapleInit(img, init_ROI);
                    is_tracking = true;
                }
            }
        } else {
            ROS_ERROR("request fail");
        }
    }
    trackerStaple(img);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("undistortFisheye", 100, callbackCamera);

    client = nh.serviceClient<ros_yolo::yolo>("yolo_service");
    client.waitForExistence(ros::Duration(30e-3));

    ros::spin();

    ros::waitForShutdown();
    return 0;
}


