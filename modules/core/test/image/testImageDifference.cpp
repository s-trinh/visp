/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Test vpImageTools::imageDifference()
 *
 *****************************************************************************/

#include <iostream>
#include <visp3/core/vpImageTools.h>

//TODO:
#include <visp3/core/vpImageFilter.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>

/*!
  \example testImageDifference.cpp

  \brief Test vpImageTools::imageDifference()
*/
namespace {
void regularImageDifference(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, vpImage<unsigned char> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images have not the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diff = I1.bitmap[b] - I2.bitmap[b] + 128;
    Idiff.bitmap[b] = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diff, 255), 0));
  }
}



//TODO:
std::vector<int> boxesForGauss(double sigma, int n) {
  double wIdeal = std::sqrt((12*sigma*sigma/n)+1);  // Ideal averaging filter width
  int wl = std::floor(wIdeal);
  if(wl%2==0) wl--;

  int wu = wl+2;

  double mIdeal = (12*sigma*sigma - n*wl*wl - 4*n*wl - 3*n)/(-4*wl - 4);
  int m = std::round(mIdeal);

  std::vector<int> sizes;
  for(int i = 0; i < n; i++) {
    sizes.push_back(i<m?wl:wu);
  }
  return sizes;
}

void boxBlurH_4 (vpImage<double>& scl, vpImage<double>& tcl, double r) {
  double iarr = 1 / (r+r+1);
  unsigned int w = scl.getWidth(), h = scl.getHeight();

  for(int i = 0; i < h; i++) {
      int ti = i*w, li = ti, ri = ti+r;
      double fv = scl.bitmap[ti], lv = scl.bitmap[ti+w-1], val = (r+1)*fv;
      for(int j = 0; j < r; j++) val += scl.bitmap[ti+j];
      for(int j = 0  ; j <= r ; j++) { val += scl.bitmap[ri++] - fv       ;   tcl.bitmap[ti++] = std::round(val*iarr); }
      for(int j = r+1; j < w-r; j++) { val += scl.bitmap[ri++] - scl.bitmap[li++];   tcl.bitmap[ti++] = std::round(val*iarr); }
      for(int j = w-r; j < w  ; j++) { val += lv        - scl.bitmap[li++];   tcl.bitmap[ti++] = std::round(val*iarr); }
  }
}

void boxBlurT_4 (const vpImage<double>& scl, vpImage<double>& tcl, double r) {
  double iarr = 1 / (r+r+1);
  unsigned int w = scl.getWidth(), h = scl.getHeight();

  for(int i = 0; i < w; i++) {
      int ti = i, li = ti, ri = ti+r*w;
      double fv = scl.bitmap[ti], lv = scl.bitmap[ti+w*(h-1)], val = (r+1)*fv;
      for(int j = 0; j < r; j++) val += scl.bitmap[ti+j*w];
      for(int j = 0  ; j <= r ; j++) { val += scl.bitmap[ri] - fv     ;  tcl.bitmap[ti] = std::round(val*iarr);  ri+=w; ti+=w; }
      for(int j = r+1; j < h-r; j++) { val += scl.bitmap[ri] - scl.bitmap[li];  tcl.bitmap[ti] = std::round(val*iarr);  li+=w; ri+=w; ti+=w; }
      for(int j = h-r; j < h  ; j++) { val += lv      - scl.bitmap[li];  tcl.bitmap[ti] = std::round(val*iarr);  li+=w; ti+=w; }
  }
}

void boxBlur_4 (vpImage<double>& scl, vpImage<double>& tcl, double radius) {
  tcl = scl;

  boxBlurH_4(tcl, scl, radius);
  boxBlurT_4(scl, tcl, radius);
}

void gaussBlur_4(vpImage<double>& scl, vpImage<double>& tcl, double radius) {
  std::vector<int> bxs = boxesForGauss(radius, 3);
//  tcl.resize(scl.getHeight(), scl.getWidth());

  boxBlur_4 (scl, tcl, (bxs[0]-1)/2.0);
  boxBlur_4 (tcl, scl, (bxs[1]-1)/2.0);
  boxBlur_4 (scl, tcl, (bxs[2]-1)/2.0);
}
}

int main()
{
//  {
//    vpImage<unsigned char> I, I_blur, I_blur_optim;
//    vpImageIo::read(I, "/home/sotrin/Documents/src/ViSP/visp-images/AprilTag/AprilTag.pgm");
//    I_blur = I;
//    I_blur_optim = I;

//    vpDisplayX d(I, 0, 0, "I"), d_blur(I_blur, 0, 0, "ViSP Gaussian blur"), d_blur_optim(I_blur_optim, 0, 0, "Optim Gaussian blur");

//    vpImage<double> I_blur_double, I_double, I_blur_double_optim;
//    vpImageConvert::convert(I, I_double);

//    double t = vpTime::measureTimeMs();
//    vpImageFilter::gaussianBlur(I, I_blur_double, 21);
//    std::cout << "ViSP Gaussian blur: " << vpTime::measureTimeMs() - t << std::endl;
//    vpImageConvert::convert(I_blur_double, I_blur);

//    t = vpTime::measureTimeMs();
//    gaussBlur_4(I_double, I_blur_double_optim, 3);
//    std::cout << "Optim Gaussian blur: " << vpTime::measureTimeMs() - t << std::endl;
//    vpImageConvert::convert(I_blur_double_optim, I_blur_optim);

//    vpDisplay::display(I);
//    vpDisplay::display(I_blur);
//    vpDisplay::display(I_blur_optim);

//    vpDisplay::flush(I);
//    vpDisplay::flush(I_blur);
//    vpDisplay::flush(I_blur_optim);
//    vpDisplay::getClick(I);


//    return 0;
//  }



  const unsigned int width = 501, height = 447;
  vpImage<unsigned char> I1(height,width), I2(height,width), Idiff_regular(height,width), Idiff_sse(height,width);
  for (unsigned int i = 0; i < I1.getRows(); i++) {
    for (unsigned int j = 0; j < I1.getCols(); j++) {
      I1[i][j] = static_cast<unsigned char>(i*256 + j);
    }
  }

  double t_regular = 0.0, t_sse = 0.0;
  for (unsigned int cpt = 0; cpt < 256; cpt++) {
    for (unsigned int i = 0; i < I2.getRows(); i++) {
      for (unsigned int j = 0; j < I2.getCols(); j++) {
        I2[i][j] = static_cast<unsigned char>(i*I2.getCols() + j + cpt);
      }
    }

    double t = vpTime::measureTimeMs();
    regularImageDifference(I1, I2, Idiff_regular);
    t_regular += vpTime::measureTimeMs() - t;

    t = vpTime::measureTimeMs();
    vpImageTools::imageDifference(I1, I2, Idiff_sse);
    t_sse += vpTime::measureTimeMs() - t;

    bool same_result = Idiff_regular == Idiff_sse;
    std::cout << "(Idiff_regular == Idiff_sse)? " << same_result << std::endl;
    if (!same_result) {
      std::cerr << "Problem with vpImageTools::imageDifference()" << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::cout << "t_regular: " << t_regular << " ms ; mean t_regular: " << t_regular/256 << " ms" << std::endl;
  std::cout << "t_sse: " << t_sse << " ms ; mean t_sse: " << t_sse/256 << " ms" << std::endl;
  std::cout << "speed-up: " << t_regular / t_sse << " X" << std::endl;

  return EXIT_SUCCESS;
}
