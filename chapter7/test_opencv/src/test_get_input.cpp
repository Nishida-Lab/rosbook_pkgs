#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

std::string g_window_name = "display";
cv::Mat g_display_image;

void on_mouse(int event, int x, int y, int flags, void* param)
{
    switch (event) {
    case CV_EVENT_MOUSEMOVE:
        g_display_image.at<cv::Vec3b>(y, x).val[1] = 255;
        g_display_image.at<cv::Vec3b>(y, x).val[2] = 255;
        break;
    case CV_EVENT_LBUTTONDOWN:
        cv::circle(g_display_image, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), 3);
        break;
    case CV_EVENT_RBUTTONDOWN:
        cv::line(g_display_image, cv::Point(x-20, y), cv::Point(x+20, y), cv::Scalar(255, 255, 0), 2);
        break;
    case CV_EVENT_RBUTTONUP:
        cv::line(g_display_image, cv::Point(x, y-20), cv::Point(x, y+20), cv::Scalar(255, 0, 255), 2);
        break;
    default:
        break;
    }
    cv::imshow(g_window_name, g_display_image);
}

int main(int argc, char* argv[])
{
    g_display_image = cv::Mat::zeros(300, 300, CV_8UC3);
    cv::namedWindow(g_window_name,
            CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cv::imshow(g_window_name, g_display_image);
    cv::setMouseCallback(g_window_name, on_mouse, 0);

    ros::init(argc, argv, "test_get_input");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);
    sensor_msgs::ImagePtr msg;

    ros::Rate looprate (30);
    while(ros::ok())
    {
        if(cv::waitKey(1) == 'q')
            break;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_display_image).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
