#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

int main()
{
  std::cout << cv::getBuildInformation() << std::endl;

  cv::Mat img = cv::Mat::ones(480, 640, CV_8U) * 255;

  cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create();
  std::vector<cv::KeyPoint> kp;
  detector->detect(img, kp);

  return 0;
}
