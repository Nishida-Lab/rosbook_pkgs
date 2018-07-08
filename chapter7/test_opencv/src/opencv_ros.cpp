#include "test_opencv/opencv_ros.hpp"
#include <time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ImageConverter::ImageConverter(ros::NodeHandle& nh){
    // Subscribe to input video feed and publish output video feed
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe("/camera/image_raw", 1,
            &ImageConverter::imageCb, this);
    image_pub_ori_ = it.advertise("/camera/image_raw", 1);
    image_pub_drawn_ = it.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter(){
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::publishReadImage(void)
{
    cv::Mat color_image = cv::imread("image.jpg", cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
    image_pub_ori_.publish(msg);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //円の描画
    cv::RNG rng(clock());
    int rand_x = (int)rng.uniform(0, cv_ptr->image.rows - 10);
    int rand_y = (int)rng.uniform(0, cv_ptr->image.cols - 10);
    cv::circle(cv_ptr->image, cv::Point(rand_x, rand_y), 10, CV_RGB(255, 0, 0));

    // GUI ウインドウのアップデート
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //結果の描画
    image_pub_drawn_.publish(cv_ptr->toImageMsg());
}
