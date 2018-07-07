#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat g_display_image;

void on_mouse(int event, int x, int y, int flags, void* param) {
    switch (event) {
    case CV_EVENT_MOUSEMOVE:
        g_display_image.at<cv::Vec3b>(y, x).val[1] = 255;
        g_display_image.at<cv::Vec3b>(y, x).val[2] = 255;
        break;
    case CV_EVENT_LBUTTONDOWN:
        cv::circle(g_display_image, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), 2);
        break;
    case CV_EVENT_RBUTTONDOWN:
        cv::line(g_display_image, cv::Point(x-20, y), cv::Point(x+20, y), cv::Scalar(255, 255, 0));
        break;
    case CV_EVENT_RBUTTONUP:
        cv::line(g_display_image, cv::Point(x, y-20), cv::Point(x, y+20), cv::Scalar(255, 0, 255));
        break;
    default:
        break;
    }
    cv::imshow("display", g_display_image);
}

int main(int argc, char* argv[]) {
    g_display_image = cv::Mat::zeros(300, 300, CV_8UC3);
    cv::imshow("display", g_display_image);
    cvSetMouseCallback("display", on_mouse, 0);

    while(1)
        if(cv::waitKey(0) == 'q')
            break;

    return 0;
}
