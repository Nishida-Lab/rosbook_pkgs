#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: rosrun test_opencv test_opencv image.jpg" << std::endl;
        exit(1);
    }

    cv::Mat source_image = cv::imread("image.jpg", 0);
    cv::imshow("image", source_image);
    cv::waitKey();

    cv::imwrite("capture.jpg", source_image);
    return 0;
}
