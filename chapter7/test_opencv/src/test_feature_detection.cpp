#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

int main(void) {
    cv::Mat color_image = cv::imread("image.jpg", 1);
    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
    
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray_image, keypoints);

    std::vector<cv::KeyPoint>::iterator it = keypoints.begin();
    for(; it!=keypoints.end(); ++it) {
        cv::circle(color_image, it->pt, 1, cv::Scalar(0, 0, 255), -1);
        cv::circle(color_image, it->pt, it->size, cv::Scalar(0, 255, 255), 1, CV_AA);
    }

    cv::imshow("Features", color_image);
    cv::waitKey(0);

    return 0;
}
