//! \example tutorial-compare-auto-gamma

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpFont.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>

// VISP_HAVE_SIMDLIB is required for INTERPOLATION_AREA
#if defined(VISP_HAVE_MODULE_IMGPROC) && defined(VISP_HAVE_SIMDLIB) && \
  ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L)))
#include <visp3/imgproc/vpImgproc.h>
#include <memory>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
template <class T>
void computeMeanMaxStdev(const vpImage<T> &I, float &mean, float &max, float &stdev)
{
  max = std::numeric_limits<float>::epsilon();
  mean = 0.;
  stdev = 0.;
  unsigned int nbRows = I.getRows();
  unsigned int nbCols = I.getCols();
  float scale = 1.f / (static_cast<float>(nbRows) * static_cast<float>(nbCols));
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      mean += I[r][c];
      max = std::max<float>(max, static_cast<float>(I[r][c]));
    }
  }
  mean *= scale;
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      stdev += (I[r][c] - mean) * (I[r][c] - mean);
    }
  }
  stdev *= scale;
  stdev = std::sqrt(stdev);
}

void computeCanny(const vpImage<unsigned char> &I, vpCannyEdgeDetection &cannyDetector, int gaussianKernelSize,
                  float gaussianStdev, int apertureSize, vpImageFilter::vpCannyFilteringAndGradientType filteringType,
                  vpImage<unsigned char> &dIxy_uchar, vpImage<unsigned char> &I_canny_visp)
{
  vpImage<float> dIx, dIy, dIxy(I.getHeight(), I.getWidth());
  int nb_iters = 1;
  vpImageFilter::computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
      apertureSize, filteringType, vpImageFilter::CANNY_VISP_BACKEND, nullptr, nb_iters); // CANNY_OPENCV_BACKEND

  for (unsigned int i = 0; i < dIx.getHeight(); i++) {
    for (unsigned int j = 0; j < dIx.getWidth(); j++) {
      dIxy[i][j] = std::sqrt(dIx[i][j]*dIx[i][j] + dIy[i][j]*dIy[i][j]);
    }
  }

  float mean, max, stdev;
  computeMeanMaxStdev(dIxy, mean, max, stdev);
  vpImageConvert::convert(dIx, dIxy_uchar);

  // Set the gradients of the vpCannyEdgeDetection
  cannyDetector.setGradients(dIx, dIy);

  I_canny_visp = cannyDetector.detect(I);
}

double computeImageEntropy(const vpImage<unsigned char> &I)
{
  // https://github.com/dengyueyun666/Image-Contrast-Enhancement/blob/cd2b1eb5bf6396e2fc3b94cd27f73933d5467147/src/Ying_2017_CAIP.cpp#L186-L207
  std::vector<int> hist(256, 0);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      int bin = I[i][j];
      hist[bin]++;
    }
  }

  double N = I.getSize();
  double cost = 0;
  for (size_t i = 0; i < hist.size(); i++) {
    if (hist[i] == 0) {
      continue;
    }
    double p = hist[i] / N;
    cost += -p * std::log2(p);
  }

  return cost;
}

double computeImageEntropy(const vpImage<double> &I)
{
  // https://github.com/dengyueyun666/Image-Contrast-Enhancement/blob/cd2b1eb5bf6396e2fc3b94cd27f73933d5467147/src/Ying_2017_CAIP.cpp#L186-L207
  std::vector<int> hist(256, 0);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      int bin = vpMath::clamp(int(255*I[i][j]), 0, 255);
      hist[bin]++;
    }
  }

  double N = I.getSize();
  double cost = 0;
  for (size_t i = 0; i < hist.size(); i++) {
    if (hist[i] == 0) {
      continue;
    }
    double p = hist[i] / N;
    cost += -p * std::log2(p);
  }

  return cost;
}

// https://stackoverflow.com/questions/7765810/is-there-a-way-to-detect-if-an-image-is-blurry/7768918#7768918
double computeImageLaplacianVar(const vpImage<unsigned char> &I)
{
  cv::Mat src;
  vpImageConvert::convert(I, src);

  // // OpenCV port of 'LAPV' algorithm (Pech2000)
  // cv::Mat lap;
  // cv::Laplacian(src, lap, CV_64F);

  // cv::Scalar mu, sigma;
  // cv::meanStdDev(lap, mu, sigma);

  // double focusMeasure = sigma.val[0]*sigma.val[0];
  // return focusMeasure;


  // OpenCV port of 'GLVN' algorithm (Santos97)
  cv::Scalar mu, sigma;
  cv::meanStdDev(src, mu, sigma);

  double focusMeasure = (sigma.val[0]*sigma.val[0]) / mu.val[0];
  return focusMeasure;
}

// https://stackoverflow.com/questions/63437029/implementing-histogram-spread-for-image-contrast-metrics/63441306#63441306
double computeImageContrast(const vpImage<unsigned char> &I)
{
  cv::Mat img;
  vpImageConvert::convert(I, img);

  // https://stackoverflow.com/questions/32952577/calculating-cumulative-histogram/48251589#48251589
  int histSize = 256;
  float range[] = { 0, 256 }; //the upper boundary is exclusive
  const float *histRange[] = { range };

  std::vector<cv::Mat> img_planes;
  img_planes.push_back(img);
  int channels[] = { 0 };
  cv::MatND hist;

  cv::calcHist(&img_planes[0], 1, channels, cv::Mat(), hist, 1, &histSize, histRange);
  cv::Mat accumulatedHist = hist.clone();
  for (int i = 1; i < histSize; i++) {
    accumulatedHist.at<float>(i) += accumulatedHist.at<float>(i - 1);
  }

  float total = img.rows * img.cols;
  for (int i = 0; i < histSize; i++) {
    accumulatedHist.at<float>(i) = 100 * accumulatedHist.at<float>(i) / total;
  }

  float B1 = 0;
  for (int i = 0; i < histSize; i++) {
    if (accumulatedHist.at<float>(i) > 25) {
      break;
    }

    B1 = i;
  }
  float B3 = 0;
  for (int i = 0; i < histSize; i++) {
    if (accumulatedHist.at<float>(i) > 75) {
      break;
    }

    B3 = i;
  }

  double min, max;
  cv::minMaxLoc(img, &min, &max);

  double contrast = (B3-B1) / (max-min);
  return contrast;
}

void process(const vpImage<unsigned char> &I, vpImage<unsigned char> &I_gamma, float gamma, vpCannyEdgeDetection &cannyDetector,
  int gaussianKernelSize, float gaussianStdev, int apertureSize, vpImageFilter::vpCannyFilteringAndGradientType filteringType,
  vpImage<unsigned char> &dIxy_uchar, vpImage<unsigned char> &I_canny_visp)
{
  visp::gammaCorrection(I, I_gamma, gamma);
  computeCanny(I_gamma, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize, filteringType, dIxy_uchar, I_canny_visp);
}

void expon(const vpImage<unsigned char> &I, vpImage<double> &I2, double a, double b)
{
  I2.resize(I.getHeight(), I.getWidth());

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      double x = I[i][j] / 255.0;
      // f = lambda x: np.exp((1-x**a)*b)
      I2[i][j] = std::exp(b * (1 - std::pow(x, a)));
    }
  }
}

// https://github.com/AndyHuang1995/Image-Contrast-Enhancement/blob/ffff894df64bd21868d98f2e39233b29a5b75d6a/ying.py#L67-L72
vpImage<double> applyK(const vpImage<unsigned char> &I, double k)
{
  // f = lambda x: np.exp((1-x**a)*b)
  // beta = f(k)
  // gamma = k**a
  // J = (I**gamma)*beta
  // return J

  const double a = -0.3293, b = 1.1258;
  vpImage<double> beta, J(I.getHeight(), I.getWidth());
  expon(I, beta, a, b);
  double gamma = 1 / std::pow(k, a);

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      J[i][j] = beta[i][j] * std::pow(I[i][j] / 255.0, gamma);
    }
  }

  return J;
}

double getGammaCorrectionBST(const vpImage<unsigned char> &I_ori, vpCannyEdgeDetection &cannyDetector, int gaussianKernelSize,
  float gaussianStdev, int apertureSize, vpImageFilter::vpCannyFilteringAndGradientType filteringType, int max_iters,
  bool auto_decimate, unsigned int max_resolution = 400)
{
  vpImage<unsigned char> I;
  if (auto_decimate) {
    const int max_decimate = 6;
    int decimate_value = 1;
    for (int decimate = 1; decimate < max_decimate; decimate++) {
      decimate_value = decimate;
      unsigned int decimate_w = I_ori.getWidth() / decimate_value;
      unsigned int decimate_h = I_ori.getHeight() / decimate_value;

      if (decimate_w <= max_resolution && decimate_h <= max_resolution) {
        break;
      }
    }

    // std::cout << "decimate_value=" << decimate_value << std::endl;
    if (decimate_value > 1) {
      vpImageTools::resize(I_ori, I, I_ori.getWidth()/decimate_value, I_ori.getHeight()/decimate_value, vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I = I_ori;
    }
  }
  else {
    I = I_ori;
  }
  vpImage<unsigned char> I_gamma = I;
  vpImage<unsigned char> dIx_uchar(I.getHeight(), I.getWidth()), dIy_uchar(I.getHeight(), I.getWidth()),
    dIxy_uchar(I.getHeight(), I.getWidth()), I_canny_visp(I.getHeight(), I.getWidth());

  double gamma_min = 1;
  double gamma_max = 20;

  cv::Mat cv_img_uchar, cv_img, edges;

  const double threshold_left_right = 5e-3;
  double gamma_current = (gamma_min + gamma_max) / 2;
  for (int i = 0; i < max_iters; i++) {
    double gamma_left = (gamma_min + gamma_current) / 2;
    double mean_left = 0;
    process(I, I_gamma, gamma_left, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize, filteringType, dIxy_uchar, I_canny_visp);
    // TODO:
    // mean_left = I_canny_visp.getMeanValue();
    // mean_left = dIxy_uchar.getMeanValue();
    mean_left = computeImageEntropy(dIxy_uchar);

    double gamma_right = (gamma_current + gamma_max) / 2;
    double mean_right = 0;
    process(I, I_gamma, gamma_right, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize, filteringType, dIxy_uchar, I_canny_visp);
    // TODO:
    // mean_right = I_canny_visp.getMeanValue();
    // mean_right = dIxy_uchar.getMeanValue();
    mean_right = computeImageEntropy(dIxy_uchar);

    if (mean_left > mean_right) {
      gamma_current = gamma_left;
      gamma_max = gamma_current + (gamma_current - gamma_left);
    }
    else {
      gamma_current = gamma_right;
      gamma_min = gamma_current - (gamma_max - gamma_current);
    }

    if (std::fabs(mean_left - mean_right) < threshold_left_right) {
      break;
    }
  }

  return gamma_current;
}

double getGammaCorrectionBSTEntropy(const vpImage<vpRGBa> &I_ori, int max_iters, bool auto_decimate,
  unsigned int max_resolution = 400, bool apply_k = false)
{
  vpImage<vpRGBa> I;
  if (auto_decimate) {
    const int max_decimate = 6;
    int decimate_value = 1;
    for (int decimate = 1; decimate < max_decimate; decimate++) {
      decimate_value = decimate;
      unsigned int decimate_w = I_ori.getWidth() / decimate_value;
      unsigned int decimate_h = I_ori.getHeight() / decimate_value;

      if (decimate_w <= max_resolution && decimate_h <= max_resolution) {
        break;
      }
    }

    // std::cout << "decimate_value=" << decimate_value << std::endl;
    if (decimate_value > 1) {
      vpImageTools::resize(I_ori, I, I_ori.getWidth()/decimate_value, I_ori.getHeight()/decimate_value, vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I = I_ori;
    }
  }
  else {
    I = I_ori;
  }
  vpImage<vpRGBa> I_gamma = I;
  vpImage<unsigned char> I_gray_gamma;

  double gamma_min = 1;
  double gamma_max = 20;

  const double threshold_left_right = 1e-6;
  double gamma_current = (gamma_min + gamma_max) / 2;
  for (int i = 0; i < max_iters; i++) {
    double gamma_left = (gamma_min + gamma_current) / 2;
    double mean_left = 0;
    visp::gammaCorrection(I, I_gamma, gamma_left);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    if (apply_k) {
      mean_left = computeImageEntropy(applyK(I_gray_gamma, gamma_left));
    }
    else {
      mean_left = computeImageEntropy(I_gray_gamma);
    }
    // std::cout << "Entropy left: " << mean_left << " ; gamma=" << gamma_left << std::endl;

    double gamma_right = (gamma_current + gamma_max) / 2;
    double mean_right = 0;
    visp::gammaCorrection(I, I_gamma, gamma_right);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    if (apply_k) {
      mean_right = computeImageEntropy(applyK(I_gray_gamma, gamma_right));
    }
    else {
      mean_right = computeImageEntropy(I_gray_gamma);
    }
    // std::cout << "Entropy right: " << mean_right << " ; gamma=" << gamma_right << std::endl;

    if (mean_left > mean_right) {
      gamma_current = gamma_left;
      gamma_max = gamma_current + (gamma_current - gamma_left);
    }
    else {
      gamma_current = gamma_right;
      gamma_min = gamma_current - (gamma_max - gamma_current);
    }

    if (std::fabs(mean_left - mean_right) < threshold_left_right) {
      break;
    }
  }

  return gamma_current;
}

double getGammaCorrectionLaplacianVar(const vpImage<vpRGBa> &I_ori, int max_iters, bool auto_decimate,
  unsigned int max_resolution = 400)
{
  vpImage<vpRGBa> I;
  if (auto_decimate) {
    const int max_decimate = 6;
    int decimate_value = 1;
    for (int decimate = 1; decimate < max_decimate; decimate++) {
      decimate_value = decimate;
      unsigned int decimate_w = I_ori.getWidth() / decimate_value;
      unsigned int decimate_h = I_ori.getHeight() / decimate_value;

      if (decimate_w <= max_resolution && decimate_h <= max_resolution) {
        break;
      }
    }

    // std::cout << "decimate_value=" << decimate_value << std::endl;
    if (decimate_value > 1) {
      vpImageTools::resize(I_ori, I, I_ori.getWidth()/decimate_value, I_ori.getHeight()/decimate_value, vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I = I_ori;
    }
  }
  else {
    I = I_ori;
  }
  vpImage<vpRGBa> I_gamma = I;
  vpImage<unsigned char> I_gray_gamma, I_gray;

  // vpImageConvert::convert(I, I_gray);
  // double laplacian_var_ori = computeImageLaplacianVar(I_gray);

  double gamma_min = 1;
  double gamma_max = 20;

  const double threshold_left_right = 1e-6;
  double gamma_current = (gamma_min + gamma_max) / 2;
  for (int i = 0; i < max_iters; i++) {
    double gamma_left = (gamma_min + gamma_current) / 2;
    double mean_left = 0;
    visp::gammaCorrection(I, I_gamma, gamma_left);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    // mean_left = computeImageLaplacianVar(I_gray_gamma) / laplacian_var_ori;
    mean_left = computeImageLaplacianVar(I_gray_gamma);

    double gamma_right = (gamma_current + gamma_max) / 2;
    double mean_right = 0;
    visp::gammaCorrection(I, I_gamma, gamma_right);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    // mean_right = computeImageLaplacianVar(I_gray_gamma) / laplacian_var_ori;
    mean_right = computeImageLaplacianVar(I_gray_gamma);

    if (mean_left > mean_right) {
      gamma_current = gamma_left;
      gamma_max = gamma_current + (gamma_current - gamma_left);
    }
    else {
      gamma_current = gamma_right;
      gamma_min = gamma_current - (gamma_max - gamma_current);
    }

    if (std::fabs(mean_left - mean_right) < threshold_left_right) {
      break;
    }
  }

  return gamma_current;
}

double getGammaCorrectionContrast(const vpImage<vpRGBa> &I_ori, int max_iters, bool auto_decimate,
  unsigned int max_resolution = 400)
{
  vpImage<vpRGBa> I;
  if (auto_decimate) {
    const int max_decimate = 6;
    int decimate_value = 1;
    for (int decimate = 1; decimate < max_decimate; decimate++) {
      decimate_value = decimate;
      unsigned int decimate_w = I_ori.getWidth() / decimate_value;
      unsigned int decimate_h = I_ori.getHeight() / decimate_value;

      if (decimate_w <= max_resolution && decimate_h <= max_resolution) {
        break;
      }
    }

    // std::cout << "decimate_value=" << decimate_value << std::endl;
    if (decimate_value > 1) {
      vpImageTools::resize(I_ori, I, I_ori.getWidth()/decimate_value, I_ori.getHeight()/decimate_value, vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I = I_ori;
    }
  }
  else {
    I = I_ori;
  }
  vpImage<vpRGBa> I_gamma = I;
  vpImage<unsigned char> I_gray_gamma, I_gray;

  double gamma_min = 1;
  double gamma_max = 20;

  const double threshold_left_right = 1e-6;
  double gamma_current = (gamma_min + gamma_max) / 2;
  for (int i = 0; i < max_iters; i++) {
    double gamma_left = (gamma_min + gamma_current) / 2;
    double mean_left = 0;
    visp::gammaCorrection(I, I_gamma, gamma_left);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    mean_left = computeImageContrast(I_gray_gamma);

    double gamma_right = (gamma_current + gamma_max) / 2;
    double mean_right = 0;
    visp::gammaCorrection(I, I_gamma, gamma_right);
    vpImageConvert::convert(I_gamma, I_gray_gamma);
    mean_right = computeImageContrast(I_gray_gamma);

    if (mean_left > mean_right) {
      gamma_current = gamma_left;
      gamma_max = gamma_current + (gamma_current - gamma_left);
    }
    else {
      gamma_current = gamma_right;
      gamma_min = gamma_current - (gamma_max - gamma_current);
    }

    if (std::fabs(mean_left - mean_right) < threshold_left_right) {
      break;
    }
  }

  return gamma_current;
}
} // namespace

int main(int argc, const char **argv)
{
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/DICM/%02d.JPG" --output "LoL_Test_results/DICM" --jpeg --lower-thresh-ratio 0.6 --upper-thresh-ratio 1.5 --gaussian-kernel-size 7 --gaussian-std 0.5
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/VV/P%07d.jpg" --output "LoL_Test_results/VV" --jpeg --half --lower-thresh-ratio 0.6 --upper-thresh-ratio 1.5 --gaussian-kernel-size 3 --gaussian-std 0.5 --downsample 4
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/DICM/%02d.JPG" --output "LoL_Test_results/DICM" --jpeg --auto-decimate

  // https://github.com/dengyueyun666/Image-Contrast-Enhancement
  // https://github.com/AndyHuang1995/Image-Contrast-Enhancement
  // https://www.researchgate.net/publication/318730125_A_New_Image_Contrast_Enhancement_Algorithm_Using_Exposure_Fusion_Framework
  // https://www.mathworks.com/matlabcentral/answers/707143-how-can-i-calculate-entropy-of-an-image-by-using-the-entropy-s-type
  // https://www.sciencedirect.com/science/article/abs/pii/S0020025523011246
  // https://www.hdm-stuttgart.de/~maucher/Python/MMCodecs/html/basicFunctions.html
  // https://stackoverflow.com/questions/40596026/what-does-entropy-mean-in-this-context
  // https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.fminbound.html
  // https://github.com/scipy/scipy/blob/main/scipy/optimize/_optimize.py
  // https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.brent.html
  // https://en.wikipedia.org/wiki/Brent%27s_method

  // 2>&1 | tee "log.txt"
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/DICM/%02d.JPG" --output "LoL_Test_results/DICM_entropy" --jpeg
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/Fusion/%d.jpg" --output "LoL_Test_results/Fusion_entropy" --jpeg
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/LIME/%d.bmp" --output "LoL_Test_results/LIME_entropy" --jpeg
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/low_rename/img_%04d.jpg" --output "LoL_Test_results/low_rename_entropy" --jpeg
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/MEF_rename/img_%04d.png" --output "LoL_Test_results/MEF_rename_entropy"
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/NPE_rename/img_%04d.jpg" --output "LoL_Test_results/NPE_rename_entropy"
  // ./tutorial-compare-auto-gamma --input "LoL_Test/Test/VV/P%07d.jpg" --output "LoL_Test_results/VV_entropy"

  // https://scikit-image.org/docs/stable/api/skimage.filters.rank.html#skimage.filters.rank.entropy
  // Exposure Control Using Bayesian Optimization Based on Entropy Weighted Image Gradient / 10.1109/ICRA.2018.8462881
  // Gradient entropy metric and p-Laplace diffusion constraint-based algorithm for noisy multispectral image fusion / https://doi.org/10.1016/j.inffus.2015.06.003
  // https://www.cse.iitm.ac.in/~vplab/courses/CV_DIP/PDF/HIST_PROC.pdf

  std::string input = "Sample_low_brightness.png";
  std::string output = "Results";
  int acquisition_step = 1;
  int gaussianKernelSize = 3;
  float gaussianStdev = 1.0f;
  int apertureSize = 3;
  bool auto_decimate = true;
  unsigned int max_decimate_resolution = 400;
  vpImageFilter::vpCannyFilteringAndGradientType filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING;
  VISP_NAMESPACE_NAME::vpGammaColorHandling gamma_colorspace = VISP_NAMESPACE_NAME::GAMMA_HSV;
  bool jpeg = false;
  int max_iters_BST = 10;
  // Canny parameters
  float lowerThresh = -1.;
  float upperThresh = -1.;
  float lowerThreshRatio = 0.6f;
  float upperThreshRatio = 0.8f;
  bool apply_k = false;
  double clip_limit = 4;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      ++i;
      input = std::string(argv[i]);
    }
    else if (std::string(argv[i]) == "--step" && i + 1 < argc) {
      ++i;
      acquisition_step = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--no-auto-decimate") {
      auto_decimate = false;
    }
    else if (std::string(argv[i]) == "--max-decimate-resolution" && i+1 < argc) {
      ++i;
      max_decimate_resolution = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--gaussian-kernel-size" && i + 1 < argc) {
      ++i;
      gaussianKernelSize = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--gaussian-std" && i + 1 < argc) {
      ++i;
      gaussianStdev = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--aperture-size" && i + 1 < argc) {
      ++i;
      apertureSize = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--canny-filtering-type" && i + 1 < argc) {
      ++i;
      int type = std::atoi(argv[i]);
      if (type == 1) {
        filteringType = vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING;
      }
    }
    else if (std::string(argv[i]) == "--gamma-rgb") {
      gamma_colorspace = VISP_NAMESPACE_NAME::GAMMA_RGB;
    }
    else if (std::string(argv[i]) == "--jpeg") {
      jpeg = true;
    }
    else if (std::string(argv[i]) == "--max-iters-BST" && i + 1 < argc) {
      ++i;
      max_iters_BST = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--lower-thresh" && i + 1 < argc) {
      ++i;
      lowerThresh = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--upper-thresh" && i + 1 < argc) {
      ++i;
      upperThresh = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--lower-thresh-ratio" && i + 1 < argc) {
      ++i;
      lowerThreshRatio = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--upper-thresh-ratio" && i + 1 < argc) {
      ++i;
      upperThreshRatio = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--apply-k") {
      apply_k = true;
    }
    else if (std::string(argv[i]) == "--CLAHE-clip-limit" && i + 1 < argc) {
      ++i;
      clip_limit = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      ++i;
      output = std::string(argv[i]);
    }
    else {
      std::cout << "Usage: " << argv[0]
        << " [--input <input path or image sequence pattern>]"
        " [--half (use half image resolution)]"
        " [--gaussian-kernel-size <e.g. 3, 5, 7>]"
        " [--gaussian-std <e.g. 1>]"
        " [--aperture-size <e.g. 3>]"
        " [--canny-filtering-type <0=CANNY_GBLUR_SOBEL_FILTERING, 1=CANNY_GBLUR_SCHARR_FILTERING>]"
        " [--gamma-rgb (RGB colorspace, else HSV)]"
        " [--jpeg (save in jpeg, otherwise png)]"
        " [--max-iters-BST"
        " [--output <folder path> (to save results)]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Input: " << input << std::endl;
  std::cout << "Acquisition step: " << acquisition_step << std::endl;
  std::cout << "Automatic decimation: " << auto_decimate << std::endl;
  std::cout << "Max decimate resolution: " << max_decimate_resolution << std::endl;
  std::cout << "Gaussian kernel size: " << gaussianKernelSize << std::endl;
  std::cout << "Gaussian standard deviation: " << gaussianStdev << std::endl;
  std::cout << "Aperture size: " << apertureSize << std::endl;
  std::cout << "Canny filtering type: " << filteringType << std::endl;
  std::cout << "RGB colorspace? " << (gamma_colorspace == VISP_NAMESPACE_NAME::GAMMA_RGB) << std::endl;
  std::cout << "Save in jpeg? " << jpeg << std::endl;
  std::cout << "Max iters BST: " << max_iters_BST << std::endl;
  std::cout << "Canny lower threshold: " << lowerThresh << std::endl;
  std::cout << "Canny upper threshold: " << upperThresh << std::endl;
  std::cout << "Canny lower threshold ratio: " << lowerThreshRatio << std::endl;
  std::cout << "Canny upper threshold ratio: " << upperThreshRatio << std::endl;
  std::cout << "Apply k? " << apply_k << std::endl;
  std::cout << "CLAHE clip limit: " << clip_limit << std::endl;
  std::cout << "Output result folder: " << output << std::endl;

  vpCannyEdgeDetection cannyDetector(gaussianKernelSize, gaussianStdev, apertureSize,
                                    lowerThresh, upperThresh, lowerThreshRatio, upperThreshRatio,
                                    filteringType);

  // TODO:
  const unsigned int max_resolution = 800;

  bool single_image = vpIoTools::checkFilename(input);
  vpVideoReader reader;
  vpImage<vpRGBa> I_color_ori, I_color;
  if (single_image) {
    vpImageIo::read(I_color_ori, input);
  }
  else {
    reader.setFrameStep(acquisition_step);
    reader.setFileName(input);
    reader.open(I_color_ori);
  }
  if (I_color_ori.getWidth() > max_resolution || I_color_ori.getHeight() > max_resolution) {
    float factor_w = I_color_ori.getWidth() / static_cast<float>(max_resolution);
    float factor_h = I_color_ori.getHeight() / static_cast<float>(max_resolution);
    int resize_factor_w = static_cast<int>(factor_w);
    int resize_factor_h = static_cast<int>(factor_h);
    int resize_factor = std::max(resize_factor_w, resize_factor_h);
    vpImageTools::resize(I_color_ori, I_color, I_color_ori.getWidth()/resize_factor, I_color_ori.getHeight()/resize_factor,
      vpImageTools::INTERPOLATION_AREA);
  }
  else {
    I_color = I_color_ori;
  }

  vpIoTools::makeDirectory(output);

  const int nb_methods = 2 + VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT - 1; // all except GAMMA_MANUAL
  std::vector<std::vector<double>> computation_times(nb_methods);
  int nb_images = 0;

  vpImage<vpRGBa> I_color_gamma_correction, I_res_stack;
  vpImage<unsigned char> I_gray, I_gray_gamma_correction, dIxy_uchar, I_canny_visp;
  vpImage<vpRGBa> dIxy_uchar_color, I_canny_visp_color;
  vpFont font(32);
  bool read_single_image = false;
  while (!read_single_image && (single_image || !reader.end())) {
    std::cout << std::endl;
    if (!single_image) {
      reader.acquire(I_color_ori);
    }
    if (I_color_ori.getWidth() > max_resolution || I_color_ori.getHeight() > max_resolution) {
      float factor_w = I_color_ori.getWidth() / static_cast<float>(max_resolution);
      float factor_h = I_color_ori.getHeight() / static_cast<float>(max_resolution);
      int resize_factor_w = static_cast<int>(factor_w);
      int resize_factor_h = static_cast<int>(factor_h);
      int resize_factor = std::max(resize_factor_w, resize_factor_h);
      vpImageTools::resize(I_color_ori, I_color, I_color_ori.getWidth()/resize_factor, I_color_ori.getHeight()/resize_factor,
        vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I_color = I_color_ori;
    }
    nb_images++;

    I_res_stack.init(nb_methods*I_color.getHeight(), 4*I_color.getWidth());
    dIxy_uchar.init(I_color.getHeight(), I_color.getWidth());
    I_canny_visp.init(I_color.getHeight(), I_color.getWidth());

    // Output results
    // int offset_text_start_y = 25;
    int offset_text_start_y = 10;
    int text_h = 40;
    int offset_idx = 0;
    double offset_text1 = 0.01;
    double offset_text2 = 0.26;
    double start_time = 0, end_time = 0;
    char buffer[FILENAME_MAX];

    vpImageConvert::convert(I_color, I_gray);

    computeCanny(I_gray, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize, filteringType, dIxy_uchar, I_canny_visp);
    const double img_ori_Canny = I_canny_visp.getMeanValue();
    const double img_ori_dIxy = dIxy_uchar.getMeanValue();
    const double img_ori_contrast = computeImageContrast(I_gray);
    const double img_ori_entropy = computeImageEntropy(I_gray);

    // BST on Canny contours
    if (false) {
      start_time = vpTime::measureTimeMs();
      double gamma_BST = getGammaCorrectionBST(I_gray, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, max_iters_BST, auto_decimate, max_decimate_resolution);
      end_time = vpTime::measureTimeMs();
      visp::gammaCorrection(I_color, I_color_gamma_correction, gamma_BST);
      std::cout << "Computation time (Gamma BST): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      snprintf(buffer, FILENAME_MAX, "gamma_BST: %.2f (%.2f ms)", gamma_BST, (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Mean Canny / dIxy: %.2f / %.2f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue());
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      offset_idx++;
    }

    // BST on image "sharpness"
    if (false) {
      start_time = vpTime::measureTimeMs();
      double gamma_BST_Laplacian = getGammaCorrectionLaplacianVar(I_color, max_iters_BST, auto_decimate, max_decimate_resolution);
      end_time = vpTime::measureTimeMs();
      visp::gammaCorrection(I_color, I_color_gamma_correction, gamma_BST_Laplacian);
      std::cout << "Computation time (Gamma BST Laplacian): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      snprintf(buffer, FILENAME_MAX, "gamma_BST_Laplacian: %.2f (%.2f ms)", gamma_BST_Laplacian, (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Mean Canny / dIxy: %.2f / %.2f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue());
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      offset_idx++;
    }

    // BST on image contrast
    if (false) {
      start_time = vpTime::measureTimeMs();
      double gamma_BST_contrast = getGammaCorrectionContrast(I_color, max_iters_BST, auto_decimate, max_decimate_resolution);
      end_time = vpTime::measureTimeMs();
      visp::gammaCorrection(I_color, I_color_gamma_correction, gamma_BST_contrast);
      std::cout << "Computation time (Gamma BST contrast): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      const double img_corrected_contrast = computeImageContrast(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Canny original
      snprintf(buffer, FILENAME_MAX, "%.2f / %.2f / %.3f", img_ori_Canny, img_ori_dIxy, img_ori_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      snprintf(buffer, FILENAME_MAX, "gamma_BST_contrast: %.2f (%.2f ms)", gamma_BST_contrast, (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Canny / dI / cont: %.2f / %.2f / %.3f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue(), img_corrected_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      offset_idx++;
    }

    // CLAHE
    {
      cv::Mat cv_img, cv_img_lab;
      vpImageConvert::convert(I_color, cv_img);
      start_time = vpTime::measureTimeMs();
      // https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c/24341809#24341809
      cv::cvtColor(cv_img, cv_img_lab, cv::COLOR_BGR2Lab);

      // Extract the L channel
      std::vector<cv::Mat> lab_planes(3);
      cv::split(cv_img_lab, lab_planes);  // now we have the L image in lab_planes[0]

      // apply the CLAHE algorithm to the L channel
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
      clahe->setClipLimit(clip_limit);
      cv::Mat dst;
      clahe->apply(lab_planes[0], dst);

      // Merge the the color planes back into an Lab image
      dst.copyTo(lab_planes[0]);
      cv::merge(lab_planes, cv_img_lab);

      // convert back to RGB
      cv::cvtColor(cv_img_lab, cv_img, CV_Lab2BGR);
      end_time = vpTime::measureTimeMs();
      vpImageConvert::convert(cv_img, I_color_gamma_correction);
      std::cout << "Computation time (CLAHE): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      const double img_corrected_contrast = computeImageContrast(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Canny original
      snprintf(buffer, FILENAME_MAX, "%.2f / %.2f / %.3f", img_ori_Canny, img_ori_dIxy, img_ori_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      snprintf(buffer, FILENAME_MAX, "CLAHE (%.2f ms)", (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Canny / dI / cont: %.2f / %.2f / %.3f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue(), img_corrected_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      offset_idx++;
    }

    {
      start_time = vpTime::measureTimeMs();
      double gamma_BST_entropy = getGammaCorrectionBSTEntropy(I_color, max_iters_BST, auto_decimate, max_resolution, apply_k);
      visp::gammaCorrection(I_color, I_color_gamma_correction, gamma_BST_entropy);
      end_time = vpTime::measureTimeMs();
      std::cout << "Computation time (Gamma BST entropy): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      const double img_corrected_contrast = computeImageContrast(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Canny original
      snprintf(buffer, FILENAME_MAX, "%.2f / %.2f / %.3f", img_ori_Canny, img_ori_dIxy, img_ori_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      snprintf(buffer, FILENAME_MAX, "gamma_BST_entropy: %.2f (%.2f ms)", gamma_BST_entropy, (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Canny / dI / cont: %.2f / %.2f / %.3f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue(), img_corrected_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
      offset_idx++;
    }

    for (int gamma_idx = 1; gamma_idx < VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT; ++gamma_idx, offset_idx++) {
      VISP_NAMESPACE_NAME::vpGammaMethod gamma_method = static_cast<VISP_NAMESPACE_NAME::vpGammaMethod>(gamma_idx);
      if (gamma_method == VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
        continue;
      }

      const double gamma = -1;
      start_time = vpTime::measureTimeMs();
      VISP_NAMESPACE_NAME::gammaCorrection(I_color, I_color_gamma_correction, static_cast<float>(gamma),
        gamma_colorspace, gamma_method);
      end_time = vpTime::measureTimeMs();
      std::cout << "Computation time (" << VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method)
        << "): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      const double img_corrected_contrast = computeImageContrast(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Canny original
      snprintf(buffer, FILENAME_MAX, "%.2f / %.2f / %.3f", img_ori_Canny, img_ori_dIxy, img_ori_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      std::ostringstream oss;
      oss <<  VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method) << " (%.2f ms)";
      snprintf(buffer, FILENAME_MAX, oss.str().c_str(), (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y,
                                                      offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Canny / dI / cont: %.2f / %.2f / %.3f", I_canny_visp.getMeanValue(), dIxy_uchar.getMeanValue(), img_corrected_contrast);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h,
                                                      offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.3f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
    }

    if (!output.empty()) {
      std::stringstream output_filename;
      const std::string extension = ".jpeg";
      if (single_image) {
        output_filename << vpIoTools::createFilePath(output, vpIoTools::getNameWE(input)) << extension;
      }
      else {
        output_filename << vpIoTools::createFilePath(output, vpIoTools::getNameWE(reader.getFrameName())) << extension;
      }
      std::cout << "Write result to: " << output_filename.str() << std::endl;
      vpImageIo::write(I_res_stack, output_filename.str());
    }

    if (single_image) {
      read_single_image = true;
    }
  }

  std::cout << "\nStats:" << std::endl;
  std::cout << "Nb images: " << nb_images << std::endl;

  std::cout << "CLAHE: mean=" << vpMath::getMean(computation_times[0]) << " ms ; median="
    << vpMath::getMedian(computation_times[0]) << " ms" << std::endl;
  std::cout << "BST (entropy): mean=" << vpMath::getMean(computation_times[1]) << " ms ; median="
    << vpMath::getMedian(computation_times[1]) << " ms" << std::endl;

  for (int gamma_idx = 1; gamma_idx < VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT; ++gamma_idx) {
    VISP_NAMESPACE_NAME::vpGammaMethod gamma_method = static_cast<VISP_NAMESPACE_NAME::vpGammaMethod>(gamma_idx);
    if (gamma_method == VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
      continue;
    }
    std::cout << VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method) << ": mean="
      << vpMath::getMean(computation_times[gamma_idx+1]) << " ms ; median="
      << vpMath::getMedian(computation_times[gamma_idx+1]) << " ms" << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "C++11 is required." << std::endl;
  return EXIT_SUCCESS;
}
#endif
