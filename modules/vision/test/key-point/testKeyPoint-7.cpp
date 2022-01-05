#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

int main()
{
  std::cout << cv::getBuildInformation() << std::endl;

  cv::Mat mask = cv::Mat::ones(img.rows, img.cols, CV_8U) * 255;
  {
    std::cout << "\nColor image:" << std::endl;
    cv::Mat img = cv::imread("/Users/runner/ViSP-images/Klimt/Klimt.ppm", cv::IMREAD_COLOR);

    cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create();
    {
      std::vector<cv::KeyPoint> kp;
      detector->detect(img, kp);
      std::cout << "1) Nb keypoints: " << kp.size() << std::endl;
    }

    {
      std::vector<cv::KeyPoint> kp;
      detector->detect(img, kp, mask);
      std::cout << "2) Nb keypoints: " << kp.size() << std::endl;
    }
  }

  {
    std::cout << "\nGrayscale image:" << std::endl;
    cv::Mat img = cv::imread("/Users/runner/ViSP-images/Klimt/Klimt.pgm", cv::IMREAD_GRAYSCALE);

    cv::Ptr<cv::FeatureDetector> detector = cv::SiftFeatureDetector::create();
    {
      std::vector<cv::KeyPoint> kp;
      detector->detect(img, kp);
      std::cout << "1) Nb keypoints: " << kp.size() << std::endl;
    }

    {
      std::vector<cv::KeyPoint> kp;
      detector->detect(img, kp, mask);
      std::cout << "2) Nb keypoints: " << kp.size() << std::endl;
    }
  }

  return 0;
}
