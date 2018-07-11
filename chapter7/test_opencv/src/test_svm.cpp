#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <iostream>

cv::Mat makeData(float offset)
{
    cv::Mat data = cv::Mat::zeros(100, 2, CV_32F);
    cv::randn(data, offset, 0.5);
    return data;
}

int main(int argc, char** argv)
{
    // (1) 学習データの生成(ディスクリプタの生成)
    cv::Mat positive_data = makeData(0.8);
    cv::Mat negative_data = makeData(-0.8);
    cv::Mat training_data;
    cv::vconcat(positive_data, negative_data, training_data);

    // (2) 学習データの生成(クラスの付与)
    cv::Mat positive_class = 1*cv::Mat::ones(positive_data.rows, 1, CV_32SC1);
    cv::Mat negative_class = 2*cv::Mat::ones(negative_data.rows, 1, CV_32SC1);
    cv::Mat training_class;
    cv::vconcat(positive_class , negative_class , training_class);
    
    // (3) 識別器の訓練
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    svm->setType(cv::ml::SVM::C_SVC);
    svm->setKernel(cv::ml::SVM::LINEAR);
    svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
    svm->train(training_data, cv::ml::ROW_SAMPLE, training_class);

    // (4) データの入力と識別処理
    std::cout<< "-1.0から1.0の範囲の数値を空白で区切って二つ入力してください";
    cv::Mat data = cv::Mat::zeros(1, 2, CV_32F);
    std::cin >> data.at<float>(0, 0) >> data.at<float>(0, 1);
    float ret = svm->predict(data);
    std::cout << "所属クラスは" << ret << "です．" << std::endl;
    
    // 結果の描画
    cv::Mat canvas= cv::Mat::zeros(200, 200, CV_8UC3);
    cv::Scalar p_color = cv::Scalar(0, 0, 255);
    cv::Scalar n_color = cv::Scalar(0, 255, 0);
    for (int i=0; i<positive_data.rows; i++)
    {
        cv::Mat p = 50*positive_data(cv::Rect(0, i, 2, 1)) + 100;
        cv::Mat n = 50*negative_data(cv::Rect(0, i, 2, 1)) + 100;
        cv::circle(canvas, cv::Point(p.at<float>(0, 0), p.at<float>(0, 1)), 1, p_color, -1);
        cv::circle(canvas, cv::Point(n.at<float>(0, 0), n.at<float>(0, 1)), 1, n_color, -1);
    }

    cv::Scalar color = (ret >= 0 ? p_color : n_color) + cv::Scalar(255, 0, 0);
    cv::Mat d = 50*data + 100;
    cv::circle(canvas, cv::Point(d.at<float>(0, 0), d.at<float>(0, 1)), 5, color, -1);

    ros::init(argc, argv, "test_svm");
    ros::NodeHandle nh;
    image_transport::Publisher image_pub;
    image_transport::ImageTransport img_trans(nh);
    image_pub = img_trans.advertise("/image_svm", 1);

    ros::Rate looprate(5);
    while (ros::ok())
    {
        if (cv::waitKey(1) == 'q')
            break;
        cv::imshow("SVM", canvas);

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
