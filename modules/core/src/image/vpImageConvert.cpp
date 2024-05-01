/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Convert image types.
 *
*****************************************************************************/

/*!
  \file vpImageConvert.cpp
  \brief Convert image types
*/

#include <map>
#include <sstream>

#if defined(_OPENMP)
#include <omp.h>
#endif


#include <visp3/core/vpConfig.h>
// image
#include "private/vpBayerConversion.h"
#include "private/vpImageConvert_impl.h"
#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif
#include <visp3/core/vpImageConvert.h>

bool vpImageConvert::YCbCrLUTcomputed = false;
int vpImageConvert::vpCrr[256];
int vpImageConvert::vpCgb[256];
int vpImageConvert::vpCgr[256];
int vpImageConvert::vpCbb[256];

/*!
  Convert a vpImage\<unsigned char\> to a vpImage\<vpRGBa\>.
  Tha alpha component is set to vpRGBa::alpha_default.
  \param[in] src : Source image
  \param[out] dest : Destination image.

  \sa GreyToRGBa()
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, vpImage<vpRGBa> &dest)
{
  dest.resize(src.getHeight(), src.getWidth());

  GreyToRGBa(src.bitmap, reinterpret_cast<unsigned char *>(dest.bitmap), src.getWidth(), src.getHeight());
}

/*!
  Convert a vpImage\<unsigned char\> to a vpImage\<vpRGBa\>
  \param[in] src : Source image
  \param[out] dest : Destination image.
  \param[in] nThreads : Number of threads to use if OpenMP is available. If 0 is passed,
  OpenMP will choose the number of threads.

  \sa RGBaToGrey()
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, vpImage<unsigned char> &dest, unsigned int nThreads)
{
  dest.resize(src.getHeight(), src.getWidth());

  RGBaToGrey(reinterpret_cast<unsigned char *>(src.bitmap), dest.bitmap, src.getWidth(), src.getHeight(), nThreads);
}

/*!
  Convert a vpImage\<float\> to a vpImage\<unsigned char\> by renormalizing
  between 0 and 255.
  \param[in] src : Source image
  \param[out] dest : Destination image.
*/
void vpImageConvert::convert(const vpImage<float> &src, vpImage<unsigned char> &dest)
{
  dest.resize(src.getHeight(), src.getWidth());
  unsigned int max_xy = src.getWidth() * src.getHeight();
  float min, max;

  src.getMinMaxValue(min, max);

  for (unsigned int i = 0; i < max_xy; ++i) {
    float val = 255.f * ((src.bitmap[i] - min) / (max - min));
    if (val < 0) {
      dest.bitmap[i] = 0;
    }
    else if (val > 255) {
      dest.bitmap[i] = 255;
    }
    else {
      dest.bitmap[i] = static_cast<unsigned char>(val);
    }
  }
}

/*!
  Convert a vpImage\<vpRGBf\> to a vpImage\<unsigned char\> by renormalizing
  between 0 and 255.
  \param[in] src : Source image
  \param[out] dest : Destination image.
*/
void vpImageConvert::convert(const vpImage<vpRGBf> &src, vpImage<vpRGBa> &dest)
{
  const unsigned int srcHeight = src.getHeight(), srcWidth = src.getWidth();
  dest.resize(src.getHeight(), src.getWidth());
  vpRGBf min, max;
  src.getMinMaxValue(min, max);

  for (unsigned int i = 0; i < srcHeight; ++i) {
    for (unsigned int j = 0; j < srcWidth; ++j) {
      for (unsigned int c = 0; c < 3; ++c) {
        float val = 255.f * ((reinterpret_cast<const float *>(&(src[i][j]))[c] - reinterpret_cast<float *>(&min)[c]) /
                             (reinterpret_cast<float *>(&max)[c] - reinterpret_cast<float *>(&min)[c]));
        if (val < 0) {
          reinterpret_cast<unsigned char *>(&(dest[i][j]))[c] = 0;
        }
        else if (val > 255) {
          reinterpret_cast<unsigned char *>(&(dest[i][j]))[c] = 255;
        }
        else {
          reinterpret_cast<unsigned char *>(&(dest[i][j]))[c] = static_cast<unsigned char>(val);
        }
      }
    }
  }
}

/*!
  Convert a vpImage\<unsigned char\> to a vpImage\<float\> by basic casting.
  \param[in] src : Source image
  \param[out] dest : Destination image.
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, vpImage<float> &dest)
{
  const unsigned int srcHeight = src.getHeight(), srcWidth = src.getWidth();
  const unsigned int srcSize = srcHeight * srcWidth;
  dest.resize(srcHeight, srcWidth);
  for (unsigned int i = 0; i < srcSize; ++i) {
    dest.bitmap[i] = static_cast<float>(src.bitmap[i]);
  }
}

/*!
  Convert a vpImage\<double\> to a vpImage\<unsigned char\> by renormalizing
  between 0 and 255.
  \param[in] src : Source image
  \param[out] dest : Destination image.
*/
void vpImageConvert::convert(const vpImage<double> &src, vpImage<unsigned char> &dest)
{
  dest.resize(src.getHeight(), src.getWidth());
  unsigned int max_xy = src.getWidth() * src.getHeight();
  double min, max;

  src.getMinMaxValue(min, max);

  for (unsigned int i = 0; i < max_xy; ++i) {
    double val = 255. * ((src.bitmap[i] - min) / (max - min));
    if (val < 0) {
      dest.bitmap[i] = 0;
    }
    else if (val > 255) {
      dest.bitmap[i] = 255;
    }
    else {
      dest.bitmap[i] = static_cast<unsigned char>(val);
    }
  }
}

/*!
  Convert a vpImage\<uint16_t> to a vpImage\<unsigned char\>.
  \param[in] src : Source image
  \param[out] dest : Destination image.
  \param[in] bitshift : Right bit shift applied to each source element.
*/
void vpImageConvert::convert(const vpImage<uint16_t> &src, vpImage<unsigned char> &dest, unsigned char bitshift)
{
  const unsigned int srcSize = src.getSize();
  dest.resize(src.getHeight(), src.getWidth());

  for (unsigned int i = 0; i < srcSize; ++i) {
    dest.bitmap[i] = static_cast<unsigned char>(src.bitmap[i] >> bitshift);
  }
}

/*!
  Convert a vpImage\<unsigned char> to a vpImage\<uint16_t\>.
  \param[in] src : Source image
  \param[out] dest : Destination image.
  \param[in] bitshift : Left bit shift applied to each source element.
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, vpImage<uint16_t> &dest, unsigned char bitshift)
{
  const unsigned int srcSize = src.getSize();
  dest.resize(src.getHeight(), src.getWidth());

  for (unsigned int i = 0; i < srcSize; ++i) {
    dest.bitmap[i] = static_cast<unsigned char>(src.bitmap[i] << bitshift);
  }
}

/*!
  Convert a vpImage\<unsigned char\> to a vpImage\<double\> by basic casting.
  \param[in] src : Source image
  \param[out] dest : Destination image.
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, vpImage<double> &dest)
{
  const unsigned int srcHeight = src.getHeight(), srcWidth = src.getWidth();
  const unsigned int srcSize = srcHeight * srcWidth;
  dest.resize(srcHeight, srcWidth);
  for (unsigned int i = 0; i < srcSize; ++i) {
    dest.bitmap[i] = static_cast<double>(src.bitmap[i]);
  }
}

/*!
  Convert the input 16-bits depth image to a color depth image. The input
  depth value is assigned a color value proportional to its frequency. Tha
  alpha component of the resulting image is set to vpRGBa::alpha_default.
  \param[in] src_depth : Input 16-bits depth image.
  \param[out] dest_rgba : Output color depth image.
*/
void vpImageConvert::createDepthHistogram(const vpImage<uint16_t> &src_depth, vpImage<vpRGBa> &dest_rgba)
{
  vp_createDepthHistogram(src_depth, dest_rgba);
}

/*!
  Convert the input 16-bits depth image to a 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input 16-bits depth image.
  \param[out] dest_depth : Output grayscale depth image.
*/
void vpImageConvert::createDepthHistogram(const vpImage<uint16_t> &src_depth, vpImage<unsigned char> &dest_depth)
{
  vp_createDepthHistogram(src_depth, dest_depth);
}

/*!
  Convert the input float depth image to a color depth image. The input
  depth value is assigned a color value proportional to its frequency. The
  alpha component of the resulting image is set to vpRGBa::alpha_default.
  \param[in] src_depth : Input float depth image.
  \param[out] dest_rgba : Output color depth image.
*/
void vpImageConvert::createDepthHistogram(const vpImage<float> &src_depth, vpImage<vpRGBa> &dest_rgba)
{
  vp_createDepthHistogram(src_depth, dest_rgba);
}

/*!
  Convert the input float depth image to a 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input float depth image.
  \param[out] dest_depth : Output grayscale depth image.
 */
void vpImageConvert::createDepthHistogram(const vpImage<float> &src_depth, vpImage<unsigned char> &dest_depth)
{
  vp_createDepthHistogram(src_depth, dest_depth);
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)

/*!
  Convert a cv::Mat to a vpImage\<vpRGBa\>.

  A cv::Mat is an OpenCV image class.

  If the input image is of type CV_8UC1 or CV_8UC3, the alpha channel is set
  to vpRGBa::alpha_default, or 0 in certain case (see the warning below).

  \warning This function is only available if OpenCV (version 2.1.0 or
  greater) was detected during the configuration step.

  \warning If ViSP is built with SSSE3 flag and the CPU supports this intrinsics set,
  alpha channel will be set to 0, otherwise it will be set to vpRGBa::alpha_default (255).

  \param[in] src : Source image in OpenCV format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] flip : Set to true to vertically flip the converted image.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/core/vpRGBa.h>
  #include <visp3/io/vpImageIo.h>

  int main()
  {
  #if defined(VISP_HAVE_OPENCV)
    vpImage<vpRGBa> Ic; // A color image
    cv::Mat Ip;

    // Read an image on a disk with openCV library
    Ip = cv::imread("image.pgm", cv::IMREAD_COLOR); // Second parameter for a BGR encoding.
    // Convert the grayscale cv::Mat into vpImage<vpRGBa>
    vpImageConvert::convert(Ip, Ic);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const cv::Mat &src, vpImage<vpRGBa> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();
  if (src.type() == CV_8UC4) {
    vpRGBa rgbaVal;
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        cv::Vec4b tmp = src.at<cv::Vec4b>(static_cast<int>(i), static_cast<int>(j));
        rgbaVal.R = tmp[2];
        rgbaVal.G = tmp[1];
        rgbaVal.B = tmp[0];
        rgbaVal.A = tmp[3];
        if (flip) {
          dest[destRows - i - 1][j] = rgbaVal;
        }
        else {
          dest[i][j] = rgbaVal;
        }
      }
  }
  else if (src.type() == CV_8UC3) {
#if defined(VISP_HAVE_SIMDLIB)
    if (src.isContinuous() && (!flip)) {
      SimdRgbToBgra(src.data, src.cols, src.rows, src.step[0], reinterpret_cast<uint8_t *>(dest.bitmap),
                    dest.getWidth() * sizeof(vpRGBa), vpRGBa::alpha_default);
    }
    else {
#endif
      vpRGBa rgbaVal;
      rgbaVal.A = vpRGBa::alpha_default;
      for (unsigned int i = 0; i < destRows; ++i) {
        for (unsigned int j = 0; j < destCols; ++j) {
          cv::Vec3b tmp = src.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j));
          rgbaVal.R = tmp[2];
          rgbaVal.G = tmp[1];
          rgbaVal.B = tmp[0];
          if (flip) {
            dest[destRows - i - 1][j] = rgbaVal;
          }
          else {
            dest[i][j] = rgbaVal;
          }
        }
      }
#if defined(VISP_HAVE_SIMDLIB)
    }
#endif
  }
  else if (src.type() == CV_8UC1) {
#if defined(VISP_HAVE_SIMDLIB)
    if (src.isContinuous() && (!flip)) {
      SimdGrayToBgra(src.data, src.cols, src.rows, src.step[0], reinterpret_cast<uint8_t *>(dest.bitmap),
                     dest.getWidth() * sizeof(vpRGBa), vpRGBa::alpha_default);
    }
    else {
#endif
      vpRGBa rgbaVal;
      for (unsigned int i = 0; i < destRows; ++i) {
        for (unsigned int j = 0; j < destCols; ++j) {
          rgbaVal = src.at<unsigned char>(static_cast<int>(i), static_cast<int>(j));
          rgbaVal.A = vpRGBa::alpha_default;
          if (flip) {
            dest[destRows - i - 1][j] = rgbaVal;
          }
          else {
            dest[i][j] = rgbaVal;
          }
        }
      }
#if defined(VISP_HAVE_SIMDLIB)
    }
#endif
  }
}

/*!
  Convert a cv::Mat to a vpImage\<unsigned char\>.

  A cv::Mat is an OpenCV image class.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param[in] src : Source image in OpenCV format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] flip : Set to true to vertically flip the converted image.
  \param[in] nThreads : number of threads to use if OpenMP is available. If 0 is passed,
  OpenMP will choose the number of threads.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  int main()
  {
  #if defined(VISP_HAVE_OPENCV)
    vpImage<unsigned char> Ig; // A grayscale image
    cv::Mat Ip;

    // Read an image on a disk with openCV library
    Ip = cv::imread("image.pgm", cv::IMREAD_GRAYSCALE); // Second parameter for a gray level.
    // Convert the grayscale cv::Mat into vpImage<unsigned char>
    vpImageConvert::convert(Ip, Ig);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const cv::Mat &src, vpImage<unsigned char> &dest, bool flip, unsigned int nThreads)
{
  if (src.type() == CV_8UC1) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous() && (!flip)) {
      memcpy(dest.bitmap, src.data, static_cast<size_t>(src.rows * src.cols));
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + ((destRows - i - 1) * src.step1()), static_cast<size_t>(src.step));
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + (i * src.step1()), static_cast<size_t>(src.step));
        }
      }
    }
  }
  else if (src.type() == CV_8UC3) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous()) {
      BGRToGrey((unsigned char *)src.data, (unsigned char *)dest.bitmap, static_cast<unsigned int>(src.cols), static_cast<unsigned int>(src.rows),
                flip, nThreads);
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRToGrey((unsigned char *)src.data + (i * src.step1()),
                    (unsigned char *)dest.bitmap + ((destRows - i - 1) * destCols),
                    static_cast<unsigned int>(destCols), 1, false);
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRToGrey((unsigned char *)src.data + (i * src.step1()), (unsigned char *)dest.bitmap + (i * destCols),
                    static_cast<unsigned int>(destCols), 1, false);
        }
      }
    }
  }
  else if (src.type() == CV_8UC4) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous()) {
      BGRaToGrey((unsigned char *)src.data, (unsigned char *)dest.bitmap, static_cast<unsigned int>(src.cols),
                 static_cast<unsigned int>(src.rows), flip, nThreads);
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRaToGrey((unsigned char *)src.data + (i * src.step1()),
                     (unsigned char *)dest.bitmap + ((destRows - i - 1) * destCols),
                     static_cast<unsigned int>(destCols), 1, false);
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRaToGrey((unsigned char *)src.data + (i * src.step1()), (unsigned char *)dest.bitmap + (i * destCols),
                     static_cast<unsigned int>(destCols), 1, false);
        }
      }
    }
  }
}

/*!
 * Converts cv::Mat CV_32FC1 format to ViSP vpImage<float>.
 *
 * \param[in] src : OpenCV image in CV_32FC1 format.
 * \param[out] dest : ViSP image in float format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<float> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();

  if (src.type() == CV_32FC1) {
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        if (flip) {
          dest[dest.getRows() - i - 1][j] = src.at<float>(static_cast<int>(i), static_cast<int>(j));
        }
        else {
          dest[i][j] = src.at<float>(static_cast<int>(i), static_cast<int>(j));
        }
      }
  }
  else {
    throw vpException(vpException::badValue, "cv::Mat type is not supported!");
  }
}

/*!
 * Converts cv::Mat CV_32FC1 format to ViSP vpImage<double>.
 *
 * \param[in] src : OpenCV image in CV_32FC1 format.
 * \param[out] dest : ViSP image in double format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<double> &dest, bool flip)
{
  vpImage<float> I_float;
  convert(src, I_float, flip);
  unsigned int nbRows = static_cast<unsigned int>(src.rows);
  unsigned int nbCols = static_cast<unsigned int>(src.cols);
  dest.resize(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      dest[i][j] = I_float[i][j];
    }
  }
}

/*!
 * Converts cv::Mat CV_16UC1 format to ViSP vpImage<uint16_t>.
 *
 * \param[in] src : OpenCV image in CV_16UC1 format.
 * \param[out] dest : ViSP image in uint16_t format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<uint16_t> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();

  if (src.type() == CV_16UC1) {
    if (src.isContinuous()) {
      memcpy(dest.bitmap, src.data, static_cast<size_t>(src.rows * src.cols) * sizeof(uint16_t));
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + ((destRows - i - 1) * src.step1() * sizeof(uint16_t)), static_cast<size_t>(src.step));
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + (i * src.step1() * sizeof(uint16_t)), static_cast<size_t>(src.step));
        }
      }
    }
  }
  else {
    throw(vpException(vpException::fatalError, "cv:Mat format not supported for conversion into vpImage<uint16_t>"));
  }
}

/*!
 * Converts cv::Mat CV_32FC3 format to ViSP vpImage<vpRGBf>.
 *
 * \param[in] src : OpenCV image in CV_32FC3 format.
 * \param[out] dest : ViSP image in vpRGBf format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<vpRGBf> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();

  if (src.type() == CV_32FC3) {
    vpRGBf rgbVal;
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        cv::Vec3f tmp = src.at<cv::Vec3f>(static_cast<int>(i), static_cast<int>(j));
        rgbVal.R = tmp[2];
        rgbVal.G = tmp[1];
        rgbVal.B = tmp[0];
        if (flip) {
          dest[destRows - i - 1][j] = rgbVal;
        }
        else {
          dest[i][j] = rgbVal;
        }
      }
  }
  else {
    throw vpException(vpException::badValue, "cv::Mat type is not supported!");
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a cv::Mat color image.

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param[in] src : Source image (vpRGBa format).
  \param[out] dest : Destination image (BGR format).

  \code
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<vpRGBa> I; // A color image
  cv::Mat Icv;

  // Read an image on a disk
  vpImageIo::read(I, "image.ppm");
  // Convert the image into color cv::Mat.
  vpImageConvert::convert(I, Icv);
  // Treatments on cv::Mat Icv
  //...
  // Save the cv::Mat on the disk
  cv::imwrite("image-cv.ppm", Icv);
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, cv::Mat &dest)
{
  cv::Mat vpToMat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC4, (void *)src.bitmap);
  cv::cvtColor(vpToMat, dest, cv::COLOR_RGBA2BGR);
}

/*!
  Convert a vpImage\<unsigned char\> to a cv::Mat grey level image.

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param[in] src : Source image.
  \param[out] dest : Destination image.
  \param[in] copyData : If true, the image is copied and modification in one
  object will not modified the other.

  \code
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_IMGCODECS)
  vpImage<unsigned char> Ig; // A grayscale image
  cv::Mat Ip;

  // Read an image on a disk
  vpImageIo::read(Ig, "image.pgm");
  // Convert the vpImage<unsigned char> in to grayscale cv::Mat
  vpImageConvert::convert(Ig, Ip);
  // Treatments on cv::Mat Ip
  //...
  // Save the cv::Mat on the disk
  cv::imwrite("image-cv.pgm", Ip);
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, cv::Mat &dest, bool copyData)
{
  if (copyData) {
    cv::Mat tmpMap(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC1, (void *)src.bitmap);
    dest = tmpMap.clone();
  }
  else {
    dest = cv::Mat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC1, (void *)src.bitmap);
  }
}

void vpImageConvert::convert(const vpImage<float> &src, cv::Mat &dest, bool copyData)
{
  if (copyData) {
    cv::Mat tmpMap(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC1, (void *)src.bitmap);
    dest = tmpMap.clone();
  }
  else {
    dest = cv::Mat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC1, (void *)src.bitmap);
  }
}

void vpImageConvert::convert(const vpImage<double> &src, cv::Mat &dest, bool copyData)
{
  unsigned int nbRows = src.getRows();
  unsigned int nbCols = src.getCols();
  vpImage<float> I_float(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      I_float[i][j] = static_cast<float>(src[i][j]);
    }
  }
  convert(I_float, dest, copyData);
}

void vpImageConvert::convert(const vpImage<vpRGBf> &src, cv::Mat &dest)
{
  cv::Mat vpToMat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC3, (void *)src.bitmap);
  cv::cvtColor(vpToMat, dest, cv::COLOR_RGB2BGR);
}

#endif

#ifdef VISP_HAVE_YARP
/*!
  Convert a vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[out] dest : Destination image in YARP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<unsigned char> I; // A mocochrome image
  // Read an image on a disk
  vpImageIo::read(I, "image.pgm");

  yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
  // Convert the vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>
  vpImageConvert::convert(I, Iyarp);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, yarp::sig::ImageOf<yarp::sig::PixelMono> *dest,
                             bool copyData)
{
  if (copyData) {
    dest->resize(src.getWidth(), src.getHeight());
    memcpy(dest->getRawImage(), src.bitmap, src.getHeight() * src.getWidth());
  }
  else {
    dest->setExternal(src.bitmap, static_cast<int>(src.getCols()), static_cast<int>(src.getRows()));
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelMono\> to a vpImage\<unsigned
  char\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
  // Read an image on a disk
  yarp::sig::file::read(*Iyarp, "image.pgm");

  // Convert the yarp::sig::ImageOf<yarp::sig::PixelMono> to a vpImage<unsigned char>
  vpImage<unsigned char> I;
  vpImageConvert::convert(Iyarp, I);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelMono> *src, vpImage<unsigned char> &dest,
                             bool copyData)
{
  dest.resize(src->height(), src->width());
  if (copyData) {
    memcpy(dest.bitmap, src->getRawImage(), src->height() * src->width() * sizeof(yarp::sig::PixelMono));
  }
  else {
    dest.bitmap = src->getRawImage();
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgba>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[in] dest : Destination image in YARP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<vpRGBa> I; // A color image
  // Read an image on a disk
  vpImageIo::read(I,"image.jpg");

  yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
  // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgba>
  vpImageConvert::convert(I,Iyarp);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgba> *dest, bool copyData)
{
  if (copyData) {
    dest->resize(src.getWidth(), src.getHeight());
    memcpy(dest->getRawImage(), src.bitmap, src.getHeight() * src.getWidth() * sizeof(vpRGBa));
  }
  else {
    dest->setExternal(src.bitmap, static_cast<int>(src.getCols()), static_cast<int>(src.getRows()));
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgba> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
  // Read an image on a disk
  yarp::sig::file::read(*Iyarp,"image.pgm");

  // Convert the yarp::sig::ImageOf<yarp::sig::PixelRgba> to a vpImage<vpRGBa>
  vpImage<vpRGBa> I;
  vpImageConvert::convert(Iyarp,I);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelRgba> *src, vpImage<vpRGBa> &dest, bool copyData)
{
  dest.resize(src->height(), src->width());
  if (copyData)
    memcpy(dest.bitmap, src->getRawImage(), src->height() * src->width() * sizeof(yarp::sig::PixelRgba));
  else {
    dest.bitmap = static_cast<vpRGBa *>(src->getRawImage());
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgb>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[out] dest : Destination image in YARP format.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<vpRGBa> I; // A color image
  // Read an image on a disk
  vpImageIo::read(I,"image.jpg");

  yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgb >();
  // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgb>
  vpImageConvert::convert(I,Iyarp);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgb> *dest)
{
  const unsigned int srcRows = src.getRows(), srcWidth = src.getWidth();
  dest->resize(src.getWidth(), src.getHeight());
  for (unsigned int i = 0; i < srcRows; ++i) {
    for (unsigned int j = 0; j < srcWidth; ++j) {
      dest->pixel(j, i).r = src[i][j].R;
      dest->pixel(j, i).g = src[i][j].G;
      dest->pixel(j, i).b = src[i][j].B;
    }
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgb> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  The alpha component of the resulting image is set to vpRGBa::alpha_default.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgb >();
  // Read an image on a disk
  yarp::sig::file::read(*Iyarp,"image.pgm");

  // Convert the yarp::sig::ImageOf<yarp::sig::PixelRgb> to a vpImage<vpRGBa>
  vpImage<vpRGBa> I;
  vpImageConvert::convert(Iyarp,I);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelRgb> *src, vpImage<vpRGBa> &dest)
{
  const int srcHeight = src->height(), srcWidth = src->width();
  dest.resize(srcHeight, srcWidth);
  for (int i = 0; i < srcHeight; ++i) {
    for (int j = 0; j < srcWidth; ++j) {
      dest[i][j].R = src->pixel(j, i).r;
      dest[i][j].G = src->pixel(j, i).g;
      dest[i][j].B = src->pixel(j, i).b;
      dest[i][j].A = vpRGBa::alpha_default;
    }
  }
}

#endif

/*!
  Convert RGB into RGBa.

  Alpha component is set to vpRGBa::alpha_default.

  \param[in] rgb : Pointer to the bitmap containing the 24-bits RGB data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::RGBToRGBa(unsigned char *rgb, unsigned char *rgba, unsigned int size)
{
  RGBToRGBa(rgb, rgba, size, 1, false);
}

/*!
  Converts a RGB image to RGBa. Alpha component is set to
  vpRGBa::alpha_default.

  Flips the image vertically if needed.
  Assumes that rgba is already resized.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] rgb : Pointer to the bitmap containing the 24-bits RGB data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.

*/
void vpImageConvert::RGBToRGBa(unsigned char *rgb, unsigned char *rgba, unsigned int width, unsigned int height,
                               bool flip)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (!flip) {
    SimdBgrToBgra(rgb, width, height, width * 3, rgba, width * 4, vpRGBa::alpha_default);
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so the
    // step is negative
    int lineStep = flip ? -static_cast<int>(width * 3) : static_cast<int>(width * 3);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (rgb + (width * height * 3) + lineStep) : rgb;

    unsigned int j = 0;
    unsigned int i = 0;

    for (i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (j = 0; j < width; ++j) {
        *rgba++ = *(++line);
        *rgba++ = *(++line);
        *rgba++ = *(++line);
        *rgba++ = vpRGBa::alpha_default;
      }
      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
}

/*!
  Convert RGB image into RGBa image.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] rgba : Pointer to the 32-bits RGBA bitmap.
  \param[out] rgb : Pointer to the bitmap containing the 24-bits RGB data that should
  be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::RGBaToRGB(unsigned char *rgba, unsigned char *rgb, unsigned int size)
{
#if defined(VISP_HAVE_SIMDLIB)
  SimdBgraToBgr(rgba, size, 1, size * 4, rgb, size * 3);
#else
  unsigned char *pt_input = rgba;
  unsigned char *pt_end = rgba + (4 * size);
  unsigned char *pt_output = rgb;

  while (pt_input != pt_end) {
    *(++pt_output) = *(++pt_input); // R
    *(++pt_output) = *(++pt_input); // G
    *(++pt_output) = *(++pt_input); // B
    pt_input++;
  }
#endif
}

/*!
  Convert an RGB image to a grey scale one.

  See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

  \param[in] rgb : Pointer to the 24-bits RGB bitmap.
  \param[out] grey : Pointer to the bitmap containing the 8-bits grey data that should
  be allocated with a size of width * height.
  \param[in] size : Image size corresponding to image width * height.

*/
void vpImageConvert::RGBToGrey(unsigned char *rgb, unsigned char *grey, unsigned int size)
{
  RGBToGrey(rgb, grey, size, 1, false);
}

/*!
  Converts a RGB image to a grey scale one.
  Flips the image vertically if needed.
  Assumes that grey is already resized.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] rgb : Pointer to the 24-bits RGB bitmap.
  \param[out] grey : Pointer to the bitmap containing the 8-bits grey data that should
  be allocated with a size of width * height.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.
*/
void vpImageConvert::RGBToGrey(unsigned char *rgb, unsigned char *grey, unsigned int width, unsigned int height,
                               bool flip)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (!flip) {
    SimdRgbToGray(rgb, width, height, width * 3, grey, width);
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so
    // the  step is negative
    int lineStep = flip ? -static_cast<int>(width * 3) : static_cast<int>(width * 3);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (rgb + (width * height * 3) + lineStep) : rgb;

    unsigned int j = 0;
    unsigned int i = 0;

    unsigned r, g, b;

    for (i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (j = 0; j < width; ++j) {
        r = *(++line);
        g = *(++line);
        b = *(++line);
        *grey++ = static_cast<unsigned char>((0.2126 * r) + (0.7152 * g) + (0.0722 * b));
      }

      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
}

/*!
  Convert a RGBa image to a grey scale one.

  See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] rgba : Pointer to the 32-bits RGBA bitmap.
  \param[out] grey : Pointer to the bitmap containing the 8-bits grey data that should
  be allocated with a size of width * height.
  \param[in] width, height : Image size.
  \param[in] nThreads : When > 0, the value is used to set the number of OpenMP threads used for the conversion.

*/
void vpImageConvert::RGBaToGrey(unsigned char *rgba, unsigned char *grey, unsigned int width, unsigned int height,
                                unsigned int
#if defined(_OPENMP)
                                nThreads
#endif
)
{
#if defined(VISP_HAVE_SIMDLIB)
  const int heightAsInt = static_cast<int>(height);
#if defined(_OPENMP)
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for
#endif
  for (int i = 0; i < heightAsInt; ++i) {
    SimdRgbaToGray(rgba + (i * width * 4), width, 1, width * 4, grey + (i * width), width);
  }
#else
#if defined(_OPENMP)
  (void)nThreads;
#endif
  vpImageConvert::RGBaToGrey(rgba, grey, width * height);
#endif
}

/*!
  Convert a RGBa image to a greyscale one.

  See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] rgba : Pointer to the 32-bits RGBA bitmap.
  \param[out] grey : Pointer to the bitmap containing the 8-bits grey data that should
  be allocated with a size of width * height.
  \param[in] size : Image size corresponding to image width * height.

*/
void vpImageConvert::RGBaToGrey(unsigned char *rgba, unsigned char *grey, unsigned int size)
{
#if defined(VISP_HAVE_SIMDLIB)
  SimdRgbaToGray(rgba, size, 1, size * 4, grey, size);
#else
  unsigned char *pt_input = rgba;
  unsigned char *pt_end = rgba + (size * 4);
  unsigned char *pt_output = grey;

  while (pt_input != pt_end) {
    *pt_output = static_cast<unsigned char>((0.2126 * (*pt_input)) + (0.7152 * (*(pt_input + 1))) + (0.0722 * (*(pt_input + 2))));
    pt_input += 4;
    pt_output++;
  }
#endif
}

/*!
  Convert from grey image to linear RGBa image.
  The alpha component is set to vpRGBa::alpha_default.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] grey : Pointer to the bitmap containing the 8-bits grey data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.
*/
void vpImageConvert::GreyToRGBa(unsigned char *grey, unsigned char *rgba, unsigned int width, unsigned int height)
{
#if defined(VISP_HAVE_SIMDLIB)
  SimdGrayToBgra(grey, width, height, width, rgba, width * sizeof(vpRGBa), vpRGBa::alpha_default);
#else
  vpImageConvert::GreyToRGBa(grey, rgba, width * height);
#endif
}

/*!
  Convert from grey image to linear RGBa image.
  The alpha component is set to vpRGBa::alpha_default.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] grey : Pointer to the bitmap containing the 8-bits grey data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.

*/
void vpImageConvert::GreyToRGBa(unsigned char *grey, unsigned char *rgba, unsigned int size)
{
#if defined(VISP_HAVE_SIMDLIB)
  GreyToRGBa(grey, rgba, size, 1);
#else
  unsigned char *pt_input = grey;
  unsigned char *pt_end = grey + size;
  unsigned char *pt_output = rgba;

  while (pt_input != pt_end) {
    unsigned char p = *pt_input;
    *(pt_output) = p;                         // R
    *(pt_output + 1) = p;                     // G
    *(pt_output + 2) = p;                     // B
    *(pt_output + 3) = vpRGBa::alpha_default; // A

    pt_input++;
    pt_output += 4;
  }
#endif
}

/*!
  Convert from grey image to linear RGB image.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] grey : Pointer to the bitmap containing the 8-bits grey data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should
  be allocated with a size of width * height * 3.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::GreyToRGB(unsigned char *grey, unsigned char *rgb, unsigned int size)
{
#if defined(VISP_HAVE_SIMDLIB)
  SimdGrayToBgr(grey, size, 1, size, rgb, size * 3);
#else
  unsigned char *pt_input = grey;
  unsigned char *pt_end = grey + size;
  unsigned char *pt_output = rgb;

  while (pt_input != pt_end) {
    unsigned char p = *pt_input;
    *(pt_output) = p;     // R
    *(pt_output + 1) = p; // G
    *(pt_output + 2) = p; // B

    pt_input++;
    pt_output += 3;
  }
#endif
}

/*!
  Converts a BGR image to RGBa. The alpha component is set to
  vpRGBa::alpha_default.

  Flips the image vertically if needed.
  Assumes that rgba is already resized.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] bgr : Pointer to the bitmap containing the 24-bits BGR data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.
*/
void vpImageConvert::BGRToRGBa(unsigned char *bgr, unsigned char *rgba, unsigned int width, unsigned int height,
                               bool flip)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (!flip) {
    SimdRgbToBgra(bgr, width, height, width * 3, rgba, width * sizeof(vpRGBa), vpRGBa::alpha_default);
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so the
    // step is negative
    int lineStep = flip ? -static_cast<int>(width * 3) : static_cast<int>(width * 3);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (bgr + (width * height * 3) + lineStep) : bgr;

    for (unsigned int i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (unsigned int j = 0; j < width; ++j) {
        *rgba++ = *(line + 2);
        *rgba++ = *(line + 1);
        *rgba++ = *(line + 0);
        *rgba++ = vpRGBa::alpha_default;

        line += 3;
      }
      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
}

/*!
  Converts a BGRa image to RGBa.

  Flips the image vertically if needed.
  Assumes that rgba is already resized before calling this function.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] bgra : Pointer to the bitmap containing the 32-bits BGRa data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.
*/
void vpImageConvert::BGRaToRGBa(unsigned char *bgra, unsigned char *rgba, unsigned int width, unsigned int height,
                                bool flip)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (!flip) {
    SimdBgraToRgba(bgra, width, height, width * 4, rgba, width * 4);
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so the
    // step is negative
    int lineStep = flip ? -static_cast<int>(width * 4) : static_cast<int>(width * 4);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (bgra + (width * height * 4) + lineStep) : bgra;

    for (unsigned int i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (unsigned int j = 0; j < width; ++j) {
        *rgba++ = *(line + 2);
        *rgba++ = *(line + 1);
        *rgba++ = *(line + 0);
        *rgba++ = *(line + 3);

        line += 4;
      }
      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
}

/*!
  Converts a BGR image to greyscale.
  Flips the image vertically if needed.
  Assumes that grey is already resized.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] bgr : Pointer to the bitmap containing the 24-bits BGR data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should
  be allocated with a size of width * height.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.
  \param[in] nThreads : When > 0, the value is used to set the number of OpenMP threads used for the conversion.
*/
void vpImageConvert::BGRToGrey(unsigned char *bgr, unsigned char *grey, unsigned int width, unsigned int height,
                               bool flip,
                               unsigned int
#if defined(_OPENMP)
                               nThreads
#endif
)
{
#if defined(VISP_HAVE_SIMDLIB)
  const int heightAsInt = static_cast<int>(height);
  if (!flip) {
#if defined(_OPENMP)
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for
#endif
    for (int i = 0; i < heightAsInt; ++i) {
      SimdBgrToGray(bgr + (i * width * 3), width, 1, width * 3, grey + (i * width), width);
    }
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so
    // the  step is negative
    int lineStep = flip ? -static_cast<int>(width * 3) : static_cast<int>(width * 3);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (bgr + (width * height * 3) + lineStep) : bgr;

    for (unsigned int i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (unsigned int j = 0; j < width; ++j) {
        *grey++ = static_cast<unsigned int>((0.2126 * *(line + 2)) + (0.7152 * *(line + 1)) + (0.0722 * *(line + 0)));
        line += 3;
      }

      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
#if !defined(VISP_HAVE_SIMDLIB) && defined(_OPENMP)
  (void)nThreads;
#endif
}

/*!
  Converts a BGRa image to greyscale.
  Flips the image vertically if needed.
  Assumes that grey is already resized.

  \note If flip is false, the SIMD lib is used to accelerate processing on x86 and ARM architecture.

  \param[in] bgra : Pointer to the bitmap containing the 32-bits BGRa data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should
  be allocated with a size of width * height.
  \param[in] width, height : Image size.
  \param[in] flip : When true, image is flipped vertically.
  \param[in] nThreads : When > 0, the value is used to set the number of OpenMP threads used for the conversion.
*/
void vpImageConvert::BGRaToGrey(unsigned char *bgra, unsigned char *grey, unsigned int width, unsigned int height,
                                bool flip,
                                unsigned int
#if defined(_OPENMP)
                                nThreads
#endif
)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (!flip) {
    const int heightAsInt = static_cast<int>(height);
#if defined(_OPENMP)
    if (nThreads > 0) {
      omp_set_num_threads(static_cast<int>(nThreads));
    }
#pragma omp parallel for
#endif
    for (int i = 0; i < heightAsInt; ++i) {
      SimdBgraToGray(bgra + (i * width * 4), width, 1, width * 4, grey + (i * width), width);
    }
  }
  else {
#endif
    // if we have to flip the image, we start from the end last scanline so
    // the  step is negative
    int lineStep = flip ? -static_cast<int>(width * 4) : static_cast<int>(width * 4);

    // starting source address = last line if we need to flip the image
    unsigned char *src = flip ? (bgra + (width * height * 4) + lineStep) : bgra;

    for (unsigned int i = 0; i < height; ++i) {
      unsigned char *line = src;
      for (unsigned int j = 0; j < width; ++j) {
        *grey++ = static_cast<unsigned char>((0.2126 * *(line + 2)) + (0.7152 * *(line + 1)) + (0.0722 * *(line + 0)));
        line += 4;
      }

      // go to the next line
      src += lineStep;
    }
#if defined(VISP_HAVE_SIMDLIB)
  }
#endif
#if !defined(VISP_HAVE_SIMDLIB) && defined(_OPENMP)
  (void)nThreads;
#endif
}

/*!
  Compute the look up table useful for YCbCr conversions.
*/
void vpImageConvert::computeYCbCrLUT()
{
  if (YCbCrLUTcomputed == false) {
    int index = 256;

    while (index--) {

      int aux = index - 128;
      vpImageConvert::vpCrr[index] = static_cast<int>((364.6610 * aux) / 256);
      vpImageConvert::vpCgb[index] = static_cast<int>((-89.8779 * aux) / 256);
      vpImageConvert::vpCgr[index] = static_cast<int>((-185.8154 * aux) / 256);
      vpImageConvert::vpCbb[index] = static_cast<int>((460.5724 * aux) / 256);
    }

    YCbCrLUTcomputed = true;
  }
}

/*!
  Convert an image from YCbCr 4:2:2 (Y0 Cb01 Y1 Cr01 Y2 Cb23 Y3 ...) to RGB
  format. Destination rgb memory area has to be allocated before.

  - In YCbCr (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGB format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue

  \param[in] ycbcr : Pointer to the bitmap containing the YCbCr 4:2:2 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should
  be allocated with a size of width * height * 3.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YCbCrToRGB(unsigned char *ycbcr, unsigned char *rgb, unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycbcr;
  unsigned char *pt_rgb = rgb;
  cbv = pt_ycbcr + 1;
  crv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    int val_r, val_g, val_b;
    if (!(col % 2)) {
      cbv = pt_ycbcr + 1;
      crv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgb++ = (val_r < 0) ? 0u : ((val_r > 255) ? 255u : static_cast<unsigned char>(val_r)); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0u : ((val_g > 255) ? 255u : static_cast<unsigned char>(val_g)); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0u : ((val_b > 255) ? 255u : static_cast<unsigned char>(val_b)); // Blue component.

    pt_ycbcr += 2;
    ++col;
  }
}

/*!
  Convert an image from YCbCr 4:2:2 (Y0 Cb01 Y1 Cr01 Y2 Cb23 Y3...) to
  RGBa format. Destination rgba memory area has to be allocated
  before.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  - In YCbCr (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGBa format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue
    Byte 3: -

  \param[in] ycbcr : Pointer to the bitmap containing the YCbCr 4:2:2 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YCbCrToRGBa(unsigned char *ycbcr, unsigned char *rgba, unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycbcr;
  unsigned char *pt_rgba = rgba;
  cbv = pt_ycbcr + 1;
  crv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    int val_r, val_g, val_b;
    if (!(col % 2)) {
      cbv = pt_ycbcr + 1;
      crv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgba++ = (val_r < 0) ? 0u : ((val_r > 255) ? 255u : static_cast<unsigned char>(val_r)); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0u : ((val_g > 255) ? 255u : static_cast<unsigned char>(val_g)); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0u : ((val_b > 255) ? 255u : static_cast<unsigned char>(val_b)); // Blue component.
    *pt_rgba++ = vpRGBa::alpha_default;

    pt_ycbcr += 2;
    ++col;
  }
}

/*!
  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to grey
  format. Destination grey image memory area has to be allocated
  before.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In grey format, each pixel is coded using 8 bytes.

  \param[in] ycbcr : Pointer to the bitmap containing the YCbCr 4:2:2 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should
  be allocated with a size of width * height.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YCbCrToGrey(unsigned char *ycbcr, unsigned char *grey, unsigned int size)
{
  unsigned int i = 0, j = 0;
  const unsigned int doubleSize = size * 2;
  while (j < doubleSize) {
    grey[i++] = ycbcr[j];
    grey[i++] = ycbcr[j + 2];
    j += 4;
  }
}

/*!
  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to RGB
  format. Destination rgb memory area has to be allocated before.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGB format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue

  \param[in] ycrcb : Pointer to the bitmap containing the YCbCr 4:2:2 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should
  be allocated with a size of width * height * 3.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YCrCbToRGB(unsigned char *ycrcb, unsigned char *rgb, unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycrcb;
  unsigned char *pt_rgb = rgb;
  crv = pt_ycbcr + 1;
  cbv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    int val_r, val_g, val_b;
    if (!(col % 2)) {
      crv = pt_ycbcr + 1;
      cbv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgb++ = (val_r < 0) ? 0u : ((val_r > 255) ? 255u : static_cast<unsigned char>(val_r)); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0u : ((val_g > 255) ? 255u : static_cast<unsigned char>(val_g)); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0u : ((val_b > 255) ? 255u : static_cast<unsigned char>(val_b)); // Blue component.

    pt_ycbcr += 2;
    ++col;
  }
}

/*!
  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to RGBa
  format. Destination rgba memory area has to be allocated before.

  The alpha component of the resulting image is set to vpRGBa::alpha_default.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGBa format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue
    Byte 3: -

  \param[in] ycrcb : Pointer to the bitmap containing the YCrCb 4:2:2 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should
  be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YCrCbToRGBa(unsigned char *ycrcb, unsigned char *rgba, unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycrcb;
  unsigned char *pt_rgba = rgba;
  crv = pt_ycbcr + 1;
  cbv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    int val_r, val_g, val_b;
    if (!(col % 2)) {
      crv = pt_ycbcr + 1;
      cbv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgba++ = (val_r < 0) ? 0u : ((val_r > 255) ? 255u : static_cast<unsigned char>(val_r)); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0u : ((val_g > 255) ? 255u : static_cast<unsigned char>(val_g)); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0u : ((val_b > 255) ? 255u : static_cast<unsigned char>(val_b)); // Blue component.
    *pt_rgba++ = vpRGBa::alpha_default;

    pt_ycbcr += 2;
    ++col;
  }
}

/*!
  Split an image from vpRGBa format to monochrome channels.
  \param[in] src : source image.
  \param[out] pR : red channel. Set as nullptr if not needed.
  \param[out] pG : green channel. Set as nullptr if not needed.
  \param[out] pB : blue channel. Set as nullptr if not needed.
  \param[out] pa : alpha channel. Set as nullptr if not needed.

  Output channels are resized if needed.

  \note The SIMD lib is used to accelerate processing on x86 and ARM architecture.

  Example code using split:

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  int main()
  {
    vpImage<vpRGBa> Ic; // A color image

    // Load a color image from the disk
    vpImageIo::read(Ic,"image.ppm");

    // Only R and B Channels are desired.
    vpImage<unsigned char> R, B;

    // Split Ic color image
    // R and B will be resized in split function if needed
    vpImageConvert::split(Ic, &R, nullptr, &B, nullptr);

    // Save the the R Channel.
    vpImageIo::write(R, "RChannel.pgm");
  }
  \endcode
*/
void vpImageConvert::split(const vpImage<vpRGBa> &src, vpImage<unsigned char> *pR, vpImage<unsigned char> *pG,
                           vpImage<unsigned char> *pB, vpImage<unsigned char> *pa)
{
#if defined(VISP_HAVE_SIMDLIB)
  if (src.getSize() > 0) {
    if (pR) {
      pR->resize(src.getHeight(), src.getWidth());
    }
    if (pG) {
      pG->resize(src.getHeight(), src.getWidth());
    }
    if (pB) {
      pB->resize(src.getHeight(), src.getWidth());
    }
    if (pa) {
      pa->resize(src.getHeight(), src.getWidth());
    }

    unsigned char *ptrR = pR ? pR->bitmap : new unsigned char[src.getSize()];
    unsigned char *ptrG = pG ? pG->bitmap : new unsigned char[src.getSize()];
    unsigned char *ptrB = pB ? pB->bitmap : new unsigned char[src.getSize()];
    unsigned char *ptrA = pa ? pa->bitmap : new unsigned char[src.getSize()];

    SimdDeinterleaveBgra(reinterpret_cast<unsigned char *>(src.bitmap), src.getWidth() * sizeof(vpRGBa), src.getWidth(),
                         src.getHeight(), ptrR, src.getWidth(), ptrG, src.getWidth(), ptrB, src.getWidth(), ptrA,
                         src.getWidth());

    if (!pR) {
      delete[] ptrR;
    }
    if (!pG) {
      delete[] ptrG;
    }
    if (!pB) {
      delete[] ptrB;
    }
    if (!pa) {
      delete[] ptrA;
    }
  }
#else
  size_t n = src.getNumberOfPixel();
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned char *input;
  unsigned char *dst;

  vpImage<unsigned char> *tabChannel[4];

  /*  incrsrc[0] = 0; //init
  incrsrc[1] = 0; //step after the first used channel
  incrsrc[2] = 0; //step after the second used channel
  incrsrc[3] = 0;
  incrsrc[4] = 0;
 */
  tabChannel[0] = pR;
  tabChannel[1] = pG;
  tabChannel[2] = pB;
  tabChannel[3] = pa;

  size_t i; /* ordre    */
  for (unsigned int j = 0; j < 4; ++j) {
    if (tabChannel[j] != nullptr) {
      if ((tabChannel[j]->getHeight() != height) || (tabChannel[j]->getWidth() != width)) {
        tabChannel[j]->resize(height, width);
      }
      dst = (unsigned char *)tabChannel[j]->bitmap;

      input = (unsigned char *)src.bitmap + j;
      i = 0;
      // optimization
      if (n >= 4) {
        n -= 3;
        for (; i < n; i += 4) {
          *dst = *input;
          input += 4;
          dst++;
          *dst = *input;
          input += 4;
          dst++;
          *dst = *input;
          input += 4;
          dst++;
          *dst = *input;
          input += 4;
          dst++;
        }
        n += 3;
      }

      for (; i < n; ++i) {
        *dst = *input;
        input += 4;
        dst++;
      }
    }
  }
#endif
}

/*!
  Merge 4 channels into an RGBa image.
  \param[in] R : Red channel.
  \param[in] G : Green channel.
  \param[in] B : Blue channel.
  \param[in] a : Alpha channel.
  \param[out] RGBa : Destination RGBa image. Image is resized internally if needed.

  \note If R, G, B, a are provided, the SIMD lib is used to accelerate processing on x86 and ARM architecture.
*/
void vpImageConvert::merge(const vpImage<unsigned char> *R, const vpImage<unsigned char> *G,
                           const vpImage<unsigned char> *B, const vpImage<unsigned char> *a, vpImage<vpRGBa> &RGBa)
{
  // Check if the input channels have all the same dimensions
  std::map<unsigned int, unsigned int> mapOfWidths, mapOfHeights;
  if (R != nullptr) {
    ++mapOfWidths[R->getWidth()];
    ++mapOfHeights[R->getHeight()];
  }

  if (G != nullptr) {
    ++mapOfWidths[G->getWidth()];
    ++mapOfHeights[G->getHeight()];
  }

  if (B != nullptr) {
    ++mapOfWidths[B->getWidth()];
    ++mapOfHeights[B->getHeight()];
  }

  if (a != nullptr) {
    ++mapOfWidths[a->getWidth()];
    ++mapOfHeights[a->getHeight()];
  }

  if ((mapOfWidths.size() == 1) && (mapOfHeights.size() == 1)) {
    unsigned int width = mapOfWidths.begin()->first;
    unsigned int height = mapOfHeights.begin()->first;

    RGBa.resize(height, width);


#if defined(VISP_HAVE_SIMDLIB)
    if ((R != nullptr) && (G != nullptr) && (B != nullptr) && (a != nullptr)) {
      SimdInterleaveBgra(R->bitmap, width, G->bitmap, width, B->bitmap, width, a->bitmap, width, width, height,
                         reinterpret_cast<uint8_t *>(RGBa.bitmap), width * sizeof(vpRGBa));
    }
    else {
#endif
      unsigned int size = width * height;
      for (unsigned int i = 0; i < size; ++i) {
        if (R != nullptr) {
          RGBa.bitmap[i].R = R->bitmap[i];
        }

        if (G != nullptr) {
          RGBa.bitmap[i].G = G->bitmap[i];
        }

        if (B != nullptr) {
          RGBa.bitmap[i].B = B->bitmap[i];
        }

        if (a != nullptr) {
          RGBa.bitmap[i].A = a->bitmap[i];
        }
      }
#if defined(VISP_HAVE_SIMDLIB)
    }
#endif
  }
  else {
    throw vpException(vpException::dimensionError, "Mismatched dimensions!");
  }
}

/*!
  Converts a MONO16 grey scale image (each pixel is coded by two bytes) into a
  grey image where each pixels are coded on one byte.

  \param[in] grey16 : Input image to convert (two bytes per pixel).
  \param[out] grey : Pointer to the 8-bit grey image (one byte per pixel) that should
  be allocated with a size of width * height.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.

*/
void vpImageConvert::MONO16ToGrey(unsigned char *grey16, unsigned char *grey, unsigned int size)
{
  int i = (static_cast<int>(size) * 2) - 1;
  int j = static_cast<int>(size) - 1;

  while (i >= 0) {
    int y = grey16[i--];
    grey[j--] = static_cast<unsigned char>((y + (grey16[i] * 256)) / 256);
    --i;
  }
}

/*!
  Converts a MONO16 grey scale image (each pixel is coded by two bytes) into a
  grey image where each pixels are coded on one byte.

  Alpha component is set to vpRGBa::alpha_default.

  \param[in] grey16 : Pointer to the bitmap containing the input image to convert (two bytes per pixel).
  \param[out] rgba : Pointer to the 32-bit RGBA image that should
  be allocated with a size of width * height * 4.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::MONO16ToRGBa(unsigned char *grey16, unsigned char *rgba, unsigned int size)
{
  int i = (static_cast<int>(size) * 2) - 1;
  int j = (static_cast<int>(size) * 4) - 1;

  while (i >= 0) {
    int y = grey16[i--];
    unsigned char v = static_cast<unsigned char>((y + (grey16[i] * 256)) / 256);
    --i;
    rgba[j--] = vpRGBa::alpha_default;
    rgba[j--] = v;
    rgba[j--] = v;
    rgba[j--] = v;
  }
}

// Bilinear

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] bggr : Array of Bayer data arranged into BGGR pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicBGGRToRGBaBilinear(const uint8_t *bggr, uint8_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicBGGRToRGBaBilinearTpl(bggr, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] bggr : Array of Bayer data arranged into BGGR pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicBGGRToRGBaBilinear(const uint16_t *bggr, uint16_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicBGGRToRGBaBilinearTpl(bggr, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] gbrg : Array of Bayer data arranged into GBRG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGBRGToRGBaBilinear(const uint8_t *gbrg, uint8_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicGBRGToRGBaBilinearTpl(gbrg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] gbrg : Array of Bayer data arranged into GBRG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGBRGToRGBaBilinear(const uint16_t *gbrg, uint16_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicGBRGToRGBaBilinearTpl(gbrg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] grbg : Array of Bayer data arranged into GRBG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGRBGToRGBaBilinear(const uint8_t *grbg, uint8_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicGRBGToRGBaBilinearTpl(grbg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] grbg : Array of Bayer data arranged into GRBG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGRBGToRGBaBilinear(const uint16_t *grbg, uint16_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicGRBGToRGBaBilinearTpl(grbg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] rggb : Array of Bayer data arranged into RGGB pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicRGGBToRGBaBilinear(const uint8_t *rggb, uint8_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicRGGBToRGBaBilinearTpl(rggb, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using bilinear demosaicing
  method.

  \param[in] rggb : Array of Bayer data arranged into RGGB pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicRGGBToRGBaBilinear(const uint16_t *rggb, uint16_t *rgba, unsigned int width,
                                                unsigned int height, unsigned int nThreads)
{
  demosaicRGGBToRGBaBilinearTpl(rggb, rgba, width, height, nThreads);
}

// Malvar

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] bggr : Array of Bayer data arranged into BGGR pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicBGGRToRGBaMalvar(const uint8_t *bggr, uint8_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicBGGRToRGBaMalvarTpl(bggr, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] bggr : Array of Bayer data arranged into BGGR pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicBGGRToRGBaMalvar(const uint16_t *bggr, uint16_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicBGGRToRGBaMalvarTpl(bggr, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] gbrg : Array of Bayer data arranged into GBRG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGBRGToRGBaMalvar(const uint8_t *gbrg, uint8_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicGBRGToRGBaMalvarTpl(gbrg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] gbrg : Array of Bayer data arranged into GBRG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGBRGToRGBaMalvar(const uint16_t *gbrg, uint16_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicGBRGToRGBaMalvarTpl(gbrg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] grbg : Array of Bayer data arranged into GRBG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGRBGToRGBaMalvar(const uint8_t *grbg, uint8_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicGRBGToRGBaMalvarTpl(grbg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] grbg : Array of Bayer data arranged into GRBG pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicGRBGToRGBaMalvar(const uint16_t *grbg, uint16_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicGRBGToRGBaMalvarTpl(grbg, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint8 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] rggb : Array of Bayer data arranged into RGGB pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicRGGBToRGBaMalvar(const uint8_t *rggb, uint8_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicRGGBToRGBaMalvarTpl(rggb, rgba, width, height, nThreads);
}

/*!
  Converts an array of uint16 Bayer data to an array of interleaved R, G, B and A values using Malvar
  \cite Malvar2004HighqualityLI demosaicing method.

  \param[in] rggb : Array of Bayer data arranged into RGGB pattern.
  \param[out] rgba : Array of R, G, B and A values converted from Bayer image.
  \param[in] width : Bayer image width.
  \param[in] height : Bayer image height.
  \param[in] nThreads : Number of OpenMP threads to use if available.

  \note rgba array must be preallocated beforehand. Alpha channel is left as-is.
*/
void vpImageConvert::demosaicRGGBToRGBaMalvar(const uint16_t *rggb, uint16_t *rgba, unsigned int width,
                                              unsigned int height, unsigned int nThreads)
{
  demosaicRGGBToRGBaMalvarTpl(rggb, rgba, width, height, nThreads);
}
