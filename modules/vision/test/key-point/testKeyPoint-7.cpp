#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

int main()
{
  std::cout << cv::getBuildInformation() << std::endl;

  cv::Mat img = cv::Mat::ones(480, 640, CV_8U) * 255;
  cv::rectangle(img, cv::Point(50,50), cv::Point(200, 200), cv::Scalar(127), -1);

  cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create();
  {
    std::vector<cv::KeyPoint> kp;
    detector->detect(img, kp);
    std::cout << "1) Nb keypoints: " << kp.size() << std::endl;
  }

  {
    std::vector<cv::KeyPoint> kp;
    cv::Mat mask = cv::Mat::ones(img.rows, img.cols, CV_8U) * 255;
    detector->detect(img, kp, mask);
    std::cout << "2) Nb keypoints: " << kp.size() << std::endl;
  }

  return 0;
}
