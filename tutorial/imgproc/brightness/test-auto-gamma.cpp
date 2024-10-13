//! \example tutorial-brightness-adjustment.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageFilter.h>

#if defined(VISP_HAVE_MODULE_IMGPROC)
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L)))
#include <memory>
#endif

namespace
{
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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

void computeCanny(const vpImage<unsigned char> &I, vpCannyEdgeDetection &cannyDetector,
  vpImage<unsigned char> &dIx_uchar, vpImage<unsigned char> &dIy_uchar, vpImage<unsigned char> &I_canny_visp,
  bool use_partial_derivative)
{
  int gaussianKernelSize = 3;
  float gaussianStdev = 1.;
  int apertureSize = 3;
  vpImageFilter::vpCannyFilteringAndGradientType filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING;

  if (use_partial_derivative) {
    vpImage<float> dIx, dIy;
    vpImageFilter::computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
        apertureSize, filteringType);

    float mean, max, stdev;
    computeMeanMaxStdev(dIx, mean, max, stdev);
    std::cout << "dIx, mean=" << mean << " ; max=" << max << " ; stdev=" << stdev << std::endl;
    vpImageConvert::convert(dIx, dIx_uchar);

    computeMeanMaxStdev(dIy, mean, max, stdev);
    std::cout << "dIy, mean=" << mean << " ; max=" << max << " ; stdev=" << stdev << std::endl;
    vpImageConvert::convert(dIy, dIy_uchar);

    // Set the gradients of the vpCannyEdgeDetection
    cannyDetector.setGradients(dIx, dIy);
  }

  // computeMeanMaxStdev(I, mean, max, stdev);
  // std::cout << "I, mean=" << mean << " ; max=" << max << " ; stdev=" << stdev << std::endl;

  I_canny_visp = cannyDetector.detect(I);
  // computeMeanMaxStdev(I_canny_visp, mean, max, stdev);
  // std::cout << "I_canny_visp, mean=" << mean << " ; max=" << max << " ; stdev=" << stdev << std::endl;
}

void process(const vpImage<unsigned char> &I, vpImage<unsigned char> &I_gamma, float gamma,
  vpCannyEdgeDetection &cannyDetector, vpImage<unsigned char> &dIx_uchar, vpImage<unsigned char> &dIy_uchar,
  vpImage<unsigned char> &I_canny_visp, bool use_partial_derivative, bool click)
{
  std::cout << "gamma=" << (1.0/gamma) << " ; 1.0/gamma=" << gamma << std::endl;

  double start_time = vpTime::measureTimeMs();
  visp::gammaCorrection(I, I_gamma, gamma);

  computeCanny(I_gamma, cannyDetector, dIx_uchar, dIy_uchar, I_canny_visp, use_partial_derivative);
  double mean = I_canny_visp.getMeanValue();
  double stdev = I_canny_visp.getStdev();
  // std::cout << "mean=" << mean << std::endl;
  double end_time = vpTime::measureTimeMs();

  vpDisplay::display(I_gamma);
  std::ostringstream oss;
  oss << "Gamma: " << 1.0/gamma << " ; Computation time: " << (end_time-start_time) << " ms";
  vpDisplay::displayText(I_gamma, 20, 20, oss.str(), vpColor::red);
  vpDisplay::flush(I_gamma);

  vpDisplay::display(dIx_uchar);
  vpDisplay::flush(dIx_uchar);

  vpDisplay::display(dIy_uchar);
  vpDisplay::flush(dIy_uchar);

  vpDisplay::display(I_canny_visp);
  oss.str("");
  oss << "Mean: " << mean << " ; Stdev: " << stdev;
  vpDisplay::displayText(I_canny_visp, 20, 20, oss.str(), vpColor::red);
  vpDisplay::flush(I_canny_visp);

  if (click) {
    vpDisplay::getClick(I_canny_visp);
  }
}
}

int main(int argc, const char **argv)
{
  std::string input = "Sample_low_brightness.png";
  int max_iters = 30;
  bool use_partial_derivative = false;
  bool click = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      ++i;
      input = std::string(argv[i]);
    }
    else if (std::string(argv[i]) == "--max-iters" && i + 1 < argc) {
      ++i;
      max_iters = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--partial-derivatives") {
      use_partial_derivative = true;
    }
    else if (std::string(argv[i]) == "--click") {
      click = true;
    }
  }
  std::cout << "input=" << input << std::endl;
  std::cout << "max_iters=" << max_iters << std::endl;
  std::cout << "use_partial_derivative=" << use_partial_derivative << std::endl;
  std::cout << "click=" << click << std::endl;


  vpImage<unsigned char> I, I_gamma;
  vpImageIo::read(I, input);
  I_gamma = I;

  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 10, 10, "Input");
  vpDisplay::display(I);
  vpDisplay::flush(I);

  // Params
  int gaussianKernelSize = 3;
  float gaussianStdev = 1.;
  int apertureSize = 3;
  vpImageFilter::vpCannyFilteringAndGradientType filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING;

  float opt_lowerThresh = -1.;
  float opt_upperThresh = -1.;
  float opt_lowerThreshRatio = 0.6f;
  float opt_upperThreshRatio = 0.8f;
  vpCannyEdgeDetection cannyDetector(gaussianKernelSize, gaussianStdev, apertureSize,
                                     opt_lowerThresh, opt_upperThresh, opt_lowerThreshRatio, opt_upperThreshRatio,
                                     filteringType);

  vpImage<unsigned char> dIx_uchar(I.getHeight(), I.getWidth()), dIy_uchar(I.getHeight(), I.getWidth()),
    I_canny_visp(I.getHeight(), I.getWidth());

  std::shared_ptr<vpDisplay> d_gamma = vpDisplayFactory::createDisplay(I_gamma, 10, 10, "Gamma");
  std::shared_ptr<vpDisplay> d_dIx = vpDisplayFactory::createDisplay(dIx_uchar, 10, 10, "dIx");
  std::shared_ptr<vpDisplay> d_dIy = vpDisplayFactory::createDisplay(dIy_uchar, 10, 10, "dIy");
  std::shared_ptr<vpDisplay> d_canny = vpDisplayFactory::createDisplay(I_canny_visp, 10, 10, "Canny");

  double gamma_min = 0.04;
  double gamma_max = 25;

  process(I, I_gamma, 1.0/gamma_min, cannyDetector, dIx_uchar, dIy_uchar, I_canny_visp, use_partial_derivative, click);
  double lower_mean = I_canny_visp.getMeanValue();
  process(I, I_gamma, 1.0/gamma_max, cannyDetector, dIx_uchar, dIy_uchar, I_canny_visp, use_partial_derivative, click);
  double upper_mean = I_canny_visp.getMeanValue();

  for (int iter = 0; iter < max_iters; iter++) {
    std::cout << "\n" << iter << ")" << std::endl;
    double range_2 = (gamma_max - gamma_min) / 2;
    if (lower_mean > upper_mean) {
      gamma_max = gamma_min + range_2;
      process(I, I_gamma, 1.0/gamma_max, cannyDetector, dIx_uchar, dIy_uchar, I_canny_visp, use_partial_derivative, click);
      upper_mean = I_canny_visp.getMeanValue();
    }
    else {
      gamma_min = gamma_max - range_2;
      process(I, I_gamma, 1.0/gamma_min, cannyDetector, dIx_uchar, dIy_uchar, I_canny_visp, use_partial_derivative, click);
      lower_mean = I_canny_visp.getMeanValue();
    }

    double diff_mean = std::fabs(upper_mean-lower_mean);
    std::cout << "gamma_min=" << gamma_min << " ; gamma_max=" << gamma_max << " ; Canny diff_mean=" << diff_mean << std::endl;
    if (diff_mean < 1e-3) {
      std::cout << "CONVERGENCE" << std::endl;
      break;
    }
  }


  double gamma_convergence = 1.0/gamma_min;
  vpImage<unsigned char> I_gray_res(I.getHeight(), 2*I.getWidth());
  visp::gammaCorrection(I, I_gamma, gamma_convergence);

  I_gray_res.insert(I, vpImagePoint(0, 0));
  I_gray_res.insert(I_gamma, vpImagePoint(0, I.getWidth()));
  vpImageIo::write(I_gray_res, "Test_auto_gamma_gray.png");

  vpImage<vpRGBa> I_color, I_color_gamma, I_color_res(I.getHeight(), 2*I.getWidth());
  vpImageIo::read(I_color, input);
  visp::gammaCorrection(I_color, I_color_gamma, gamma_convergence);

  I_color_res.insert(I_color, vpImagePoint(0, 0));
  I_color_res.insert(I_color_gamma, vpImagePoint(0, I_color.getWidth()));
  vpImageIo::write(I_color_res, "Test_auto_gamma_color.png");

  return EXIT_SUCCESS;
}
