#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(void) {
    cv::Mat source_image = cv::imread("image.jpg", 0);
    cv::Mat edge_image;
    cv::Sobel(source_image, edge_image, CV_32F, 1, 1);
    cv::imshow("sobel", edge_image);
    cv::waitKey();

    return 0;
}
