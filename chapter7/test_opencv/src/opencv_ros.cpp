#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "ImageWindow";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_drawn_;

public:
    ImageConverter(ros::NodeHandle& nh)
    {
        // Subscribe to input video feed and publish output video feed
        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe("/camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ori_ = it.advertise("/camera/image_raw", 1);
        image_pub_drawn_ = it.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    };

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    };

    void publishReadImage(void)
    {
        std::string file_path = ros::package::getPath("test_opencv") + "/img/image1.jpg";
        cv::Mat color_image = cv::imread(file_path, cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        image_pub_ori_.publish(msg);
    };
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        //円の描画
        cv::RNG rng(clock());
        int circle_r = 20;
        int rand_x = (int)rng.uniform(0, cv_ptr->image.cols - circle_r*2);
        int rand_y = (int)rng.uniform(0, cv_ptr->image.rows - circle_r*2);
        std::cout << rand_x << ", " << rand_y << std::endl;
        cv::circle(cv_ptr->image, cv::Point(rand_x, rand_y), circle_r, CV_RGB(255, 0, 0), -1);

        // GUI ウインドウのアップデート
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        //結果の描画
        image_pub_drawn_.publish(cv_ptr->toImageMsg());
    };
};

int main(int argc , char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    ImageConverter ic(nh);

    ros::Rate looprate (5);   // read image at 5Hz
    while(ros::ok())
    {
        ic.publishReadImage();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
