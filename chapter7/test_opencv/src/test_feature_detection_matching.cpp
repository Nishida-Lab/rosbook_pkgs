#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

void DetectFeature(cv::Mat& source_image,
        std::vector<cv::KeyPoint> keypoints, cv::Mat& descriptors) {
    cv::Mat gray_image;
    cv::cvtColor(source_image, gray_image, CV_BGR2GRAY);

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
    
    detector->detectAndCompute(gray_image, cv::Mat(), keypoints, descriptors);
    /*
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor =
        cv::xfeatures2d::SURF::create();

    descriptor_extractor->compute(gray_image , keypoints, descriptors);
    */
}

int main(void) {
    cv::Mat color_image = cv::imread("image.jpg", 1);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    DetectFeature(color_image, keypoints, descriptors);

    cv::Mat color_image2 = cv::imread("image2.jpg", 1);
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    DetectFeature(color_image2, keypoints2, descriptors2);

    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher =
        cv::DescriptorMatcher::create("BruteForce");
    //cv::FlannBasedMatcher descriptor_matcher;

    if ((descriptors.empty()) || (descriptors2.empty()))
    {
        printf("Descriptors empty");
        return 1;
    }

    std::vector<cv::DMatch> matches;
    descriptor_matcher->match(descriptors, descriptors2, matches);

    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors.rows; i++ )
    { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    cv::Mat result_image;
    cv::drawMatches(color_image, keypoints, color_image2, keypoints2, matches, result_image);
    cv::imshow("Matching result", result_image);
    cv::waitKey(0);

    return 0;
}
