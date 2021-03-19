//
// Created by chrisliu on 2021/3/19.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "video_publisher_node");
    ros::NodeHandle n;

    ros::Publisher video_pub = n.advertise<sensor_msgs::Image>("video_publisher",1);
    cv::VideoCapture capture;
    capture.open("/home/chrisliu/ROSws/YoloStaple_ws/video.mp4");
    cv::Mat frame;
    while (capture.isOpened()) {
        capture.read(frame);

        sensor_msgs::Image img = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        video_pub.publish(img);
        cv::imshow("Img",frame);
        cv::waitKey(1);

        ros::spinOnce();
    }
    return 0;
}

