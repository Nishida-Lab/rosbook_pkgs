#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

int main(int argc, char** argv)
{
    std::string file_path = ros::package::getPath("test_opencv") + "/img/image1.jpg";
    cv::Mat color_image = cv::imread(file_path, cv::IMREAD_COLOR);
    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

    cv::Ptr<cv::MSER> detector = cv::MSER::create();
    
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray_image, keypoints);

    std::vector<cv::KeyPoint>::iterator it = keypoints.begin();
    for(; it!=keypoints.end(); ++it) {
        cv::circle(color_image, it->pt, 1, cv::Scalar(0, 0, 255), -1);
        cv::circle(color_image, it->pt, it->size, cv::Scalar(0, 255, 255), 1, CV_AA);
    }

    ros::init(argc, argv, "test_feature_detection");
    ros::NodeHandle nh;
    image_transport::Publisher image_pub;
    image_transport::ImageTransport img_trans(nh);
    image_pub = img_trans.advertise("/image_feature", 1);

    ros::Rate looprate (5);   // read image at 5Hz
    while (ros::ok())
    {
        if(cv::waitKey(1) == 'q')
            break;
        cv::imshow("Features", color_image);

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
