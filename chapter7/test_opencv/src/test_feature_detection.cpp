//#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

int main(void) {
    cv::Mat color_image = cv::imread("image.jpg", 1);
    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);

    cv::Ptr<cv::MSER> ms = cv::MSER::create();
    //cv::MSER detector;
    
    std::vector<cv::Point> keypoints;
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(gray_image, regions, mser_bbox);

    std::cout << "start" << std::endl;
    std::cout << keypoints.size() << std::endl;

    std::vector<cv::Point>::iterator it = keypoints.begin();

    int i = 0;
    for(; it!=keypoints.end(); ++it) {
        std::cout << *it << std::endl;
        std::cout << i++ << std::endl;
        cv::circle(color_image, *it, 1, cv::Scalar(0, 0, 255), -1);
        cv::circle(color_image, *it, 3/*it->size*/, cv::Scalar(0, 255, 255), 1, CV_AA);
    }
    cv::imshow("Features", color_image);
    cv::waitKey(0);

    return 0;
}
