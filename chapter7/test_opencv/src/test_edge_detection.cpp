#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

const std::string g_window_name = "sobel";
const std::string g_file_path = ros::package::getPath("test_opencv") + "/img/image3.jpg";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_edge_;
    cv::Mat image_ori_;

public:
    ImageConverter(ros::NodeHandle& nh)
    {
        image_transport::ImageTransport it(nh);
        image_pub_ori_ = it.advertise("/image_ori", 1);
        image_pub_edge_ = it.advertise("/image_edge", 1);

        cv::namedWindow(g_window_name,
                CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    };

    ~ImageConverter()
    {
        cv::destroyWindow(g_window_name);
    };

    void publishEdgeImage(cv::Mat& edge_image)
    {
        cv::Mat image_ori_ = cv::imread(g_file_path, cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg_ori =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_ori_).toImageMsg();
        image_pub_ori_.publish(msg_ori);

        cv::Mat image_edge_8u;
        edge_image.convertTo(image_edge_8u, CV_8U);
        sensor_msgs::ImagePtr msg_edge =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", image_edge_8u).toImageMsg();
        image_pub_edge_.publish(msg_edge);
    };
};

int main(int argc, char* argv[])
{
    cv::Mat source_image = cv::imread(g_file_path, cv::IMREAD_GRAYSCALE);
    cv::Mat edge_image;
    cv::Sobel(source_image, edge_image, CV_32F, 1, 1, 3);

    ros::init(argc, argv, "test_edge_detection");
    ros::NodeHandle nh;
    ImageConverter ic(nh);

    ros::Rate looprate (5);
    while(ros::ok())
    {
        if(cv::waitKey(1) == 'q')
            break;
        cv::imshow(g_window_name, edge_image);

        ic.publishEdgeImage(edge_image);
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
