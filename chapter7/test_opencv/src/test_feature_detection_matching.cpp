#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

void detectFeature(cv::Mat& source_image,
        std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    cv::Mat gray_image;
    cv::cvtColor(source_image, gray_image, CV_BGR2GRAY);

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
    detector->detectAndCompute(source_image, cv::Mat(), keypoints, descriptors);
}

void extractGoodMatches(cv::Mat descriptors1,
        std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& good_matches)
{
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptors1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    for( int i = 0; i < descriptors1.rows; i++ )
    { if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
        { good_matches.push_back( matches[i]); }
    }
}

int main(int argc, char** argv)
{
    std::string file_path1 = ros::package::getPath("test_opencv") + "/img/image2.jpg";
    cv::Mat color_image1 = cv::imread(file_path1, cv::IMREAD_COLOR);
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    detectFeature(color_image1, keypoints1, descriptors1);

    std::string file_path2 = ros::package::getPath("test_opencv") + "/img/image1.jpg";
    cv::Mat color_image2 = cv::imread(file_path2, cv::IMREAD_COLOR);
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    detectFeature(color_image2, keypoints2, descriptors2);

    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher =
        cv::DescriptorMatcher::create("BruteForce");

    std::vector<cv::DMatch> matches, good_matches;
    descriptor_matcher->match(descriptors1, descriptors2, matches);

    extractGoodMatches(descriptors1, matches, good_matches);

    cv::Mat result_image;
    cv::drawMatches(color_image1, keypoints1, color_image2, keypoints2,
            good_matches, result_image, cv::Scalar::all(-1), cv::Scalar::all(-1),
            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    ros::init(argc, argv, "test_feature_detection_matching");
    ros::NodeHandle nh;
    image_transport::Publisher image_pub;
    image_transport::ImageTransport img_trans(nh);
    image_pub = img_trans.advertise("/image_feature_matching", 1);

    ros::Rate looprate (5);
    while (ros::ok()) {
        if(cv::waitKey(1) == 'q')
            break;
        cv::imshow("Matching result", result_image);

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
