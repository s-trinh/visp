/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test for image conversions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <stdlib.h>
#include <iomanip>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTime.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#  include <emmintrin.h>
#  define VISP_HAVE_SSE2 1

#  if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <pmmintrin.h>
#    define VISP_HAVE_SSE3 1
#  endif
#  if defined __SSSE3__  || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <tmmintrin.h>
#    define VISP_HAVE_SSSE3 1
//Cannot detect AVX with QtCreator
#    include <immintrin.h>
#    define VISP_HAVE_AVX 1
#  endif
#endif

#if VISP_HAVE_AVX
namespace {
//  const __m256i K0 = _mm256_setr_epi8(
//      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70,
//      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0);

//  const __m256i K1 = _mm256_setr_epi8(
//      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
//      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70);

//  inline const __m256i custom_mm256_shuffle_epi8(const __m256i & value, const __m256i & shuffle) {
//    return _mm256_or_si256(_mm256_shuffle_epi8(value, _mm256_add_epi8(shuffle, K0)),
//                           _mm256_shuffle_epi8(_mm256_permute4x64_epi64(value, 0x4E), _mm256_add_epi8(shuffle, K1)));
//  }



  //Workaround to be able to shuffle with cross lane
//  const __m256i K0 = _mm256_setr_epi8(
//      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70,
//      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0);
//  const __m256i K1 = _mm256_setr_epi8(
//      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
//      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70);


  const __m256i K0 = _mm256_set_epi8(
      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70,
      0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70);

  const __m256i K1 = _mm256_set_epi8(
      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
      0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0);

  inline const __m256i custom_mm256_shuffle_epi8(const __m256i & value, const __m256i & shuffle) {
//    return _mm256_or_si256(_mm256_shuffle_epi8(value, shuffle/*_mm256_add_epi8(shuffle, K0)*/),
//                           _mm256_shuffle_epi8(_mm256_permute4x64_epi64(value, 0x4E), shuffle/*_mm256_add_epi8(shuffle, K1)*/));

    return _mm256_or_si256( _mm256_shuffle_epi8(value, _mm256_adds_epu8(shuffle, K0)),
                            _mm256_shuffle_epi8(_mm256_permute4x64_epi64(value, 0x4E), _mm256_add_epi8(shuffle, K1))
          );

//    return _mm256_shuffle_epi8(value, _mm256_adds_epu8(shuffle, K0));
  }
}
#endif

/*!
  \example testConversion.cpp

  \brief Manipulation of image conversions.
*/

// List of allowed command line options
#define GETOPTARGS	"cdi:o:n:h"


/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Test image conversions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>] [-n <nb benchmark iterations>]\n\
     [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/Klimt/Klimt.pgm\"\n\
     and \"ViSP-images/Klimt/Klimt.ppm\" images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     Klimt_grey.pgm and Klimt_color.ppm output images\n\
     are written.\n\
\n\
  -n <nb benchmark iterations>                               %s\n\
     Set the number of benchmark iterations.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \param nbIterations : Number of benchmark iterations.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user,
                int &nbIterations)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i': ipath = optarg_; break;
    case 'o': opath = optarg_; break;
    case 'n': nbIterations = atoi(optarg_); break;
    case 'h': usage(argv[0], NULL, ipath, opath, user); return false; break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

void computeRegularRGBaToGrayscale(const unsigned char* rgba, unsigned char* grey, unsigned int size) {
  const unsigned char *pt_input = rgba;
  const unsigned char *pt_end = rgba + size*4;
  unsigned char *pt_output = grey;

  while(pt_input != pt_end) {
    *pt_output = (unsigned char) (0.2126 * (*pt_input)
      + 0.7152 * (*(pt_input + 1))
      + 0.0722 * (*(pt_input + 2)) );
    pt_input += 4;
    pt_output ++;
  }
}

void computeRegularRGBToGrayscale(const unsigned char* rgb, unsigned char* grey, unsigned int size) {
  const unsigned char *pt_input = rgb;
  const unsigned char* pt_end = rgb + size*3;
  unsigned char *pt_output = grey;

  while(pt_input != pt_end) {
    *pt_output = (unsigned char) (0.2126 * (*pt_input)
      + 0.7152 * (*(pt_input + 1))
      + 0.0722 * (*(pt_input + 2)) );
    pt_input += 3;
    pt_output ++;
  }
}

void computeRegularBGRToGrayscale(unsigned char * bgr, unsigned char * grey,
                                  unsigned int width, unsigned int height, bool flip) {
  //if we have to flip the image, we start from the end last scanline so the
  //step is negative
  int lineStep = (flip) ? -(int)(width*3) : (int)(width*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? bgr+(width*height*3)+lineStep : bgr;
  unsigned char * line;

  unsigned int j=0;
  unsigned int i=0;

  for(i=0 ; i < height ; i++)
  {
    line = src;
    for( j=0 ; j < width ; j++)
    {
      *grey++ = (unsigned char)( 0.2126 * *(line+2)
         + 0.7152 * *(line+1)
         + 0.0722 * *(line+0)) ;
      line+=3;
    }

    //go to the next line
    src+=lineStep;
  }
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
void computeRegularBGRToGrayscale(const cv::Mat& src, vpImage<unsigned char>& dest)
{
  if(src.type() == CV_8UC3) {
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);

    if(src.isContinuous()) {
      computeRegularBGRToGrayscale((unsigned char*)src.data, (unsigned char*)dest.bitmap, (unsigned int)src.cols, (unsigned int)src.rows, false);
    }
  }
}
#endif

void test_SSE(const unsigned char *bgr, unsigned char *grey, const unsigned int width, const unsigned int height) {
  //Mask to select B component
  const __m128i mask_B1 = _mm_set_epi8(
        -1, -1, -1, -1, 15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1
        );
  const __m128i mask_B2 = _mm_set_epi8(
        5, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m128i mask_B3 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 14, -1, 11, -1, 8, -1
        );
  const __m128i mask_B4 = _mm_set_epi8(
        13, -1, 10, -1, 7, -1, 4, -1, 1, -1, -1, -1, -1, -1, -1, -1
        );

  //Mask to select G component
  const __m128i mask_G1 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, 13, -1, 10, -1, 7, -1, 4, -1, 1, -1
        );
  const __m128i mask_G2 = _mm_set_epi8(
        6, -1, 3, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m128i mask_G3 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 15, -1, 12, -1, 9, -1
        );
  const __m128i mask_G4 = _mm_set_epi8(
        14, -1, 11, -1, 8, -1, 5, -1, 2, -1, -1, -1, -1, -1, -1, -1
        );

  //Mask to select R component
  const __m128i mask_R1 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, 14, -1, 11, -1, 8, -1, 5, -1, 2, -1
        );
  const __m128i mask_R2 = _mm_set_epi8(
        7, -1, 4, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m128i mask_R3 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 13, -1, 10, -1
        );
  const __m128i mask_R4 = _mm_set_epi8(
        15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1, -1, -1, -1, -1
        );

  //Mask to select the gray component
  const __m128i mask_low1 = _mm_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, 15, 13, 11, 9, 7, 5, 3, 1
        );
  const __m128i mask_low2 = _mm_set_epi8(
        15, 13, 11, 9, 7, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1
        );

  //Coefficients RGB to Gray
  const __m128i coeff_R = _mm_set_epi16(
        13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933
        );
  const __m128i coeff_G = _mm_set_epi16(
        46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871
        );
  const __m128i coeff_B = _mm_set_epi16(
        4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732
        );

  unsigned int i = 0;
  unsigned int size = width * height;

  if(size >= 16) {
    for(; i <= size - 16; i+=16) {
      //Process 16 color pixels
      const __m128i data1 = _mm_loadu_si128((const __m128i*) bgr);
      const __m128i data2 = _mm_loadu_si128((const __m128i*) (bgr + 16));
      const __m128i data3 = _mm_loadu_si128((const __m128i*) (bgr + 32));

//      unsigned char char_data1[16];
//      _mm_storeu_si128( (__m128i*) char_data1, data1 );
//      for(int cpt = 0; cpt < 16; cpt++) {
//        std::cout << "char_data1[" << cpt << "]=" << static_cast<unsigned>(char_data1[cpt]) << std::endl;
//      }

      const __m128i red_0_7 = _mm_or_si128( _mm_shuffle_epi8(data1, mask_R1), _mm_shuffle_epi8(data2, mask_R2) );
      const __m128i green_0_7 = _mm_or_si128( _mm_shuffle_epi8(data1, mask_G1), _mm_shuffle_epi8(data2, mask_G2) );
      const __m128i blue_0_7 = _mm_or_si128( _mm_shuffle_epi8(data1, mask_B1), _mm_shuffle_epi8(data2, mask_B2) );

//      unsigned char char_blue_0_7[16];
//      _mm_storeu_si128( (__m128i*) char_blue_0_7, blue_0_7 );
//      for(int cpt = 0; cpt < 16; cpt++) {
//        std::cout << "char_blue_0_7[" << cpt << "]=" << static_cast<unsigned>(char_blue_0_7[cpt]) << std::endl;
//      }

//      unsigned char char_green_0_7[16];
//      _mm_storeu_si128( (__m128i*) char_green_0_7, green_0_7 );
//      for(int cpt = 0; cpt < 16; cpt++) {
//        std::cout << "char_green_0_7[" << cpt << "]=" << static_cast<unsigned>(char_green_0_7[cpt]) << std::endl;
//      }

      unsigned char char_red_0_7[16];
      _mm_storeu_si128( (__m128i*) char_red_0_7, red_0_7 );
      for(int cpt = 0; cpt < 16; cpt++) {
        std::cout << "char_red_0_7[" << cpt << "]=" << static_cast<unsigned>(char_red_0_7[cpt]) << std::endl;
      }

      const __m128i grays_0_7 =
          _mm_adds_epu16(
            _mm_mulhi_epu16(red_0_7, coeff_R),
            _mm_adds_epu16(
              _mm_mulhi_epu16(green_0_7, coeff_G),
              _mm_mulhi_epu16(blue_0_7,  coeff_B)
              ));

      const __m128i red_8_15 = _mm_or_si128( _mm_shuffle_epi8(data2, mask_R3), _mm_shuffle_epi8(data3, mask_R4) );
      const __m128i green_8_15 = _mm_or_si128( _mm_shuffle_epi8(data2, mask_G3), _mm_shuffle_epi8(data3, mask_G4) );
      const __m128i blue_8_15 = _mm_or_si128( _mm_shuffle_epi8(data2, mask_B3), _mm_shuffle_epi8(data3, mask_B4) );

//      unsigned char char_blue_8_15[16];
//      _mm_storeu_si128( (__m128i*) char_blue_8_15, blue_8_15 );
//      for(int cpt = 0; cpt < 16; cpt++) {
//        std::cout << "char_blue_8_15[" << cpt << "]=" << static_cast<unsigned>(char_blue_8_15[cpt]) << std::endl;
//      }

//      unsigned char char_green_8_15[16];
//      _mm_storeu_si128( (__m128i*) char_green_8_15, green_8_15 );
//      for(int cpt = 0; cpt < 16; cpt++) {
//        std::cout << "char_green_8_15[" << cpt << "]=" << static_cast<unsigned>(char_green_8_15[cpt]) << std::endl;
//      }

      unsigned char char_red_8_15[16];
      _mm_storeu_si128( (__m128i*) char_red_8_15, red_8_15 );
      for(int cpt = 0; cpt < 16; cpt++) {
        std::cout << "char_red_8_15[" << cpt << "]=" << static_cast<unsigned>(char_red_8_15[cpt]) << std::endl;
      }

      const __m128i grays_8_15 =
          _mm_adds_epu16(
            _mm_mulhi_epu16(red_8_15, coeff_R),
            _mm_adds_epu16(
              _mm_mulhi_epu16(green_8_15, coeff_G),
              _mm_mulhi_epu16(blue_8_15,  coeff_B)
              ));

      _mm_storeu_si128( (__m128i*) grey,_mm_or_si128(_mm_shuffle_epi8(grays_0_7, mask_low1), _mm_shuffle_epi8(grays_8_15, mask_low2)) );

      bgr += 48;
      grey += 16;

      //Break
      break;
    }
  }

//  for(; i < size; i++) {
//    *grey = (unsigned char) (0.2126 * (*(bgr + 2))
//                             + 0.7152 * (*(bgr + 1))
//                             + 0.0722 * (*bgr) );

//    bgr += 3;
//    ++grey;
//  }
}

void test_AVX(const unsigned char *bgr, unsigned char *grey, const unsigned int width, const unsigned int height) {
//  //Mask to select B component
//  const __m256i mask_B1 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 30, -1, 27, -1, 24, -1, 21, -1, 18, -1, 15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, 1
//        );
//  const __m256i mask_B2 = _mm256_set_epi8(
//        13, -1, 10, -1, 7, -1, 4, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );
//  const __m256i mask_B3 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, -1, 28, -1, 25, -1, 22, -1, 19, -1, 16, -1
//        );
//  const __m256i mask_B4 = _mm256_set_epi8(
//        29, -1, 26, -1, 23, -1, 20, -1, 17, -1, 14, -1, 11, -1, 8, -1, 5, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );

//  //Mask to select G component
//  const __m256i mask_G1 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, -1, 28, -1, 25, -1, 22, -1, 19, -1, 16, -1, 13, -1, 10, -1, 7, -1, 4, -1, 1, -1
//        );
//  const __m256i mask_G2 = _mm256_set_epi8(
//        14, -1, 11, -1, 8, -1, 5, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );
//  const __m256i mask_G3 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 29, -1, 26, -1, 23, -1, 20, -1, 17, -1
//        );
//  const __m256i mask_G4 = _mm256_set_epi8(
//        30, -1, 27, -1, 24, -1, 21, -1, 18, -1, 15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );

//  //Mask to select R component
//  const __m256i mask_R1 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 29, -1, 26, -1, 23, -1, 20, -1, 17, -1, 14, -1, 11, -1, 8, -1, 5, -1, 2, -1
//        );
//  const __m256i mask_R2 = _mm256_set_epi8(
//        15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );
//  const __m256i mask_R3 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 30, -1, 27, -1, 24, -1, 21, -1, 18, -1
//        );
//  const __m256i mask_R4 = _mm256_set_epi8(
//        31, -1, 28, -1, 25, -1, 22, -1, 19, -1, 16, -1, 13, -1, 10, -1, 7, -1, 4, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );

//  //Mask to select the gray component
//  const __m256i mask_low1 = _mm256_set_epi8(
//        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1
//        );
//  const __m256i mask_low2 = _mm256_set_epi8(
//        31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
//        );



  //Mask to select B component
  const __m256i mask_B1 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 14, -1, 11, -1, 8, -1, 21, -1, 18, -1, 15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1
        );
  const __m256i mask_B2 = _mm256_set_epi8(
        29, -1, 26, -1, 23, -1, 20, -1, 17, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m256i mask_B3 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, -1, 28, -1, 25, -1, 22, -1, 19, -1, 16, -1
        );
  const __m256i mask_B4 = _mm256_set_epi8(
        13, -1, 10, -1, 7, -1, 4, -1, 1, -1, 30, -1, 27, -1, 24, -1, 5, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );

  //Mask to select G component
  const __m256i mask_G1 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 15, -1, 12, -1, 9, -1, 22, -1, 19, -1, 16, -1, 13, -1, 10, -1, 7, -1, 4, -1, 1, -1
        );
  const __m256i mask_G2 = _mm256_set_epi8(
        30, -1, 27, -1, 24, -1, 21, -1, 18, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m256i mask_G3 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 29, -1, 26, -1, 23, -1, 20, -1, 17, -1
        );
  const __m256i mask_G4 = _mm256_set_epi8(
        14, -1, 11, -1, 8, -1, 5, -1, 2, -1, 31, -1, 28, -1, 25, -1, 6, -1, 3, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );

  //Mask to select R component
  const __m256i mask_R1 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 13, -1, 10, -1, 23, -1, 20, -1, 17, -1, 14, -1, 11, -1, 8, -1, 5, -1, 2, -1
        );
  const __m256i mask_R2 = _mm256_set_epi8(
        31, -1, 28, -1, 25, -1, 22, -1, 19, -1, 16, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );
  const __m256i mask_R3 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 30, -1, 27, -1, 24, -1, 21, -1, 18, -1
        );
  const __m256i mask_R4 = _mm256_set_epi8(
        15, -1, 12, -1, 9, -1, 6, -1, 3, -1, 0, -1, 29, -1, 26, -1, 7, -1, 4, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );

  //Mask to select the gray component
  const __m256i mask_low1 = _mm256_set_epi8(
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1
        );
  const __m256i mask_low2 = _mm256_set_epi8(
        15, 13, 11, 9, 7, 5, 3, 1, 31, 29, 27, 25, 23, 21, 19, 17, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        );

  //Coefficients RGB to Gray
  const __m256i coeff_R = _mm256_set_epi16(
        13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933, 13933
        );
  const __m256i coeff_G = _mm256_set_epi16(
        46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871, 46871
        );
  const __m256i coeff_B = _mm256_set_epi16(
        4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732, 4732
        );

  unsigned int i = 0;
  unsigned int size = width * height;

  if(size >= 32) {
    for(; i <= size - 32; i+=32) {
      //Process 32 color pixels
      const __m256i data1 = _mm256_lddqu_si256((const __m256i*) bgr);
      const __m256i data2 = _mm256_lddqu_si256((const __m256i*) (bgr + 32));
      const __m256i data3 = _mm256_lddqu_si256((const __m256i*) (bgr + 64));

      unsigned char char_data1[32];
      _mm256_storeu_si256( (__m256i*) char_data1, data1 );
      for(int cpt = 0; cpt < 32; cpt++) {
        std::cout << "char_data1[" << cpt << "]=" << static_cast<unsigned>(char_data1[cpt]) << std::endl;
      }

      const __m256i red_0_15 = _mm256_or_si256( custom_mm256_shuffle_epi8(data1, mask_R1), custom_mm256_shuffle_epi8(data2, mask_R2) );
      const __m256i green_0_15 = _mm256_or_si256( custom_mm256_shuffle_epi8(data1, mask_G1), custom_mm256_shuffle_epi8(data2, mask_G2) );
      const __m256i blue_0_15 = _mm256_or_si256( custom_mm256_shuffle_epi8(data1, mask_B1), custom_mm256_shuffle_epi8(data2, mask_B2) );

//      unsigned char char_blue1_0_15[32], char_blue2_0_15[32], char_blue_0_15[32];
//      _mm256_storeu_si256( (__m256i*) char_blue1_0_15, custom_mm256_shuffle_epi8(data1, mask_B1) );
//      _mm256_storeu_si256( (__m256i*) char_blue2_0_15, custom_mm256_shuffle_epi8(data2, mask_B2) );
//      _mm256_storeu_si256( (__m256i*) char_blue_0_15, blue_0_15 );
//      for(int cpt = 0; cpt < 32; cpt++) {
//        std::cout << "char_blue1_0_15[" << cpt << "]=" << static_cast<unsigned>(char_blue1_0_15[cpt]) << " ; ";
//        std::cout << "char_blue2_0_15[" << cpt << "]=" << static_cast<unsigned>(char_blue2_0_15[cpt]) << " ; ";
//        std::cout << "char_blue_0_15[" << cpt << "]=" << static_cast<unsigned>(char_blue_0_15[cpt]) << std::endl;
//      }

//      unsigned char char_green1_0_15[32], char_green2_0_15[32], char_green_0_15[32];
//      _mm256_storeu_si256( (__m256i*) char_green1_0_15, custom_mm256_shuffle_epi8(data1, mask_G1) );
//      _mm256_storeu_si256( (__m256i*) char_green2_0_15, custom_mm256_shuffle_epi8(data2, mask_G2) );
//      _mm256_storeu_si256( (__m256i*) char_green_0_15, green_0_15 );
//      for(int cpt = 0; cpt < 32; cpt++) {
//        std::cout << "char_green1_0_15[" << cpt << "]=" << static_cast<unsigned>(char_green1_0_15[cpt]) << " ; ";
//        std::cout << "char_green2_0_15[" << cpt << "]=" << static_cast<unsigned>(char_green2_0_15[cpt]) << " ; ";
//        std::cout << "char_green_0_15[" << cpt << "]=" << static_cast<unsigned>(char_green_0_15[cpt]) << std::endl;
//      }

      unsigned char char_red1_0_15[32], char_red2_0_15[32], char_red_0_15[32];
      _mm256_storeu_si256( (__m256i*) char_red1_0_15, custom_mm256_shuffle_epi8(data1, mask_R1) );
      _mm256_storeu_si256( (__m256i*) char_red2_0_15, custom_mm256_shuffle_epi8(data2, mask_R2) );
      _mm256_storeu_si256( (__m256i*) char_red_0_15, red_0_15 );
      for(int cpt = 0; cpt < 32; cpt++) {
        std::cout << "char_red1_0_15[" << cpt << "]=" << static_cast<unsigned>(char_red1_0_15[cpt]) << " ; ";
        std::cout << "char_red2_0_15[" << cpt << "]=" << static_cast<unsigned>(char_red2_0_15[cpt]) << " ; ";
        std::cout << "char_red_0_15[" << cpt << "]=" << static_cast<unsigned>(char_red_0_15[cpt]) << std::endl;
      }

      const __m256i grays_0_15 =
          _mm256_adds_epu16(
            _mm256_mulhi_epu16(red_0_15, coeff_R),
            _mm256_adds_epu16(
              _mm256_mulhi_epu16(green_0_15, coeff_G),
              _mm256_mulhi_epu16(blue_0_15,  coeff_B)
              ));

      const __m256i red_16_31 = _mm256_or_si256( custom_mm256_shuffle_epi8(data2, mask_R3), custom_mm256_shuffle_epi8(data3, mask_R4) );
      const __m256i green_16_31 = _mm256_or_si256( custom_mm256_shuffle_epi8(data2, mask_G3), custom_mm256_shuffle_epi8(data3, mask_G4) );
      const __m256i blue_16_31 = _mm256_or_si256( custom_mm256_shuffle_epi8(data2, mask_B3), custom_mm256_shuffle_epi8(data3, mask_B4) );

      const __m256i grays_16_31 =
          _mm256_adds_epu16(
            _mm256_mulhi_epu16(red_16_31, coeff_R),
            _mm256_adds_epu16(
              _mm256_mulhi_epu16(green_16_31, coeff_G),
              _mm256_mulhi_epu16(blue_16_31,  coeff_B)
              ));

      _mm256_storeu_si256( (__m256i*) grey,_mm256_or_si256(custom_mm256_shuffle_epi8(grays_0_15, mask_low1), custom_mm256_shuffle_epi8(grays_16_31, mask_low2)) );

      bgr += 96;
      grey += 32;

      //Break
      break;
    }
  }

  //Test first part
  unsigned char char_mask_B1[32];
  _mm256_storeu_si256( (__m256i*) char_mask_B1, mask_B1 );
  for(int cpt = 0; cpt < 32; cpt++) {
    std::cout << "char_mask_B1[" << cpt << "]=" << std::hex << static_cast<unsigned>(char_mask_B1[cpt]) << std::dec << std::endl;
  }

  const __m256i K3 = _mm256_setr_epi8(
      0, -1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, -1, 31);

  __m256i part1 = _mm256_adds_epu8(K3 /*mask_B1*/, K0); //_mm256_and_si256(_mm256_add_epi8(mask_B1, K0), K3);
  unsigned char char_part1[32];
  _mm256_storeu_si256( (__m256i*) char_part1, part1 );
  for(int cpt = 0; cpt < 32; cpt++) {
    std::cout << "char_part1[" << cpt << "]=" << std::hex <<  static_cast<unsigned>(char_part1[cpt]) << std::dec << std::endl;
  }

  __m256i part2 = _mm256_add_epi8(K3 /*mask_B1*/, K1); //_mm256_and_si256(_mm256_add_epi8(mask_B1, K0), K3);
  unsigned char char_part2[32];
  _mm256_storeu_si256( (__m256i*) char_part2, part2 );
  for(int cpt = 0; cpt < 32; cpt++) {
    std::cout << "char_part2[" << cpt << "]=" << std::hex <<  static_cast<unsigned>(char_part2[cpt]) << std::dec << std::endl;
  }


//  for(; i < size; i++) {
//    *grey = (unsigned char) (0.2126 * (*(bgr + 2))
//                             + 0.7152 * (*(bgr + 1))
//                             + 0.0722 * (*bgr) );

//    bgr += 3;
//    ++grey;
//  }
}

int
main(int argc, const char ** argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string opt_opath;
    std::string ipath;
    std::string opath;
    std::string filename;
    std::string username;
    int nbIterations = 100;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (! env_ipath.empty())
      ipath = env_ipath;

    // Set the default output path
#if defined(_WIN32)
    opt_opath = "C:/temp";
#else
    opt_opath = "/tmp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_opath, username, nbIterations) == false) {
      exit (-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;
    if (!opt_opath.empty())
      opath = opt_opath;

    // Append to the output path string, the login name of the user
    opath = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(opath) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(opath);
      }
      catch (...) {
        usage(argv[0], NULL, ipath, opt_opath, username);
        std::cerr << std::endl
                  << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << opath << std::endl;
        std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
        exit(-1);
      }
    }

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (opt_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl
                  << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()){
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
                << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl << std::endl;
      exit(-1);
    }

    //
    // Here starts really the test
    //

    vpImage<unsigned char> Ig ; // Grey image
    vpImage<vpRGBa> Ic ; // Color image

    //-------------------- .pgm -> .ppm
    vpTRACE("Convert a grey image (.pgm) to a color image (.ppm)");
    // Load a grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");
    vpCTRACE << "Load " <<  filename << std::endl;
    vpImageIo::read(Ig, filename) ;
    // Create a color image from the grey
    vpImageConvert::convert(Ig, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color.ppm");
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(Ic, filename) ;

    //-------------------- .ppm -> .pgm
    vpTRACE("Convert a color image (.ppm) to a grey image (.pgm)");
    // Load a color image from the disk
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    // Create a grey image from the color
    vpImageConvert::convert(Ic, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey.pgm");
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(Ig, filename) ;

    //-------------------- YUV -> RGB
    unsigned char y=187, u=10, v=30;
    unsigned char r, g, b;

    // Convert a YUV pixel value to a RGB value
    vpImageConvert::YUVToRGB(y, u, v, r, g, b);
    vpTRACE("y(%d) u(%d) v(%d) = r(%d) g(%d) b(%d)", y, u, v, r, g, b);

#ifdef VISP_HAVE_OPENCV
#if VISP_HAVE_OPENCV_VERSION < 0x020408
    double t0 = vpTime::measureTimeMs();
    /////////////////////////
    // Convert a IplImage to a vpImage<vpRGBa>
    ////////////////////////
    IplImage* image = NULL; /*!< The image read / acquired */
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cv.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cv.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ///////////////////////////
    // Convert a IplImage to a vpImage<unsigned char>
    ////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(image, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cv.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    if(image!=NULL) cvReleaseImage( &image );
    if((image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;

      return (-1);
    }
    vpImageConvert::convert(image, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cv.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> to a IplImage
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImageConvert::convert(Ic, image);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_color_cv.ppm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    if((cvSaveImage(filename.c_str(), image)) == 0) {
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      if(image!=NULL) cvReleaseImage( &image );
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////////
    // Convert a IplImage to a vpImage<unsigned char>
    ////////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the grey image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ig, filename) ;
    vpImageConvert::convert(Ig, image);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_grey_cv.pgm");
    /* Save the the current image */

    vpCTRACE << "Write " << filename << std::endl;
    if((cvSaveImage(filename.c_str(), image)) == 0) {
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      if(image!=NULL) cvReleaseImage( &image );
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    if(image!=NULL) cvReleaseImage( &image );
    double t1 = vpTime::measureTimeMs();
    std::cout << "Conversion c interface : " << t1 - t0 << " ms" << std::endl;
#endif

    /* ------------------------------------------------------------------------ */
    /*                  conversion for the new c++ interface                    */
    /* ------------------------------------------------------------------------ */

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    double t2 = vpTime::measureTimeMs();
    /////////////////////////
    // Convert a cv::Mat to a vpImage<vpRGBa>
    ////////////////////////
    cv::Mat imageMat;
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
    vpCTRACE << "Reading the color image with c++ interface of opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 1);// force to a three channel color image.
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return -1;
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");
    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 0);// forced to grayscale.
    if(imageMat.data == NULL) {
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(imageMat, Ic);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.ppm");
    /* Save the the current image */
    vpImageIo::write(Ic, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ///////////////////////////
    // Convert a cv::Mat to a vpImage<unsigned char>
    ////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    vpCTRACE << "Reading the color image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 1);// force to a three channel color image.
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return -1;
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_color_cvMat.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the pgm image */

    vpCTRACE << "Reading the greyscale image with opencv: "<< std::endl
             << filename << std::endl;
    imageMat = cv::imread(filename, 0);
    if(imageMat.data == NULL){
      vpCTRACE<<"Cannot read image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpImageConvert::convert(imageMat, Ig);
    filename = vpIoTools::createFilePath(opath, "Klimt_grey_cvMat.pgm");
    /* Save the the current image */
    vpImageIo::write(Ig, filename) ;

    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> to a cv::Mat
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImageConvert::convert(Ic, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_color_cvMat.ppm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    if(!cv::imwrite(filename, imageMat)){
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////////
    // Convert a IplImage to a vpImage<unsigned char>
    ////////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    /* Read the grey image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ig, filename);
    vpImageConvert::convert(Ig, imageMat);
    filename = vpIoTools::createFilePath(opath, "Klimt_ipl_grey_cvMat.pgm");
    /* Save the the current image */

    vpCTRACE << "Write " << filename << std::endl;
    if(!cv::imwrite(filename, imageMat)){
      vpCTRACE<<"Cannot write image: "<< std::endl << filename << std::endl;
      return (-1);
    }
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;
    double t3 = vpTime::measureTimeMs();
    std::cout << "Conversion c++ interface : " << t3 - t2 << " ms" << std::endl;
#endif
#endif

    ////////////////////////////////////
    // Split a vpImage<vpRGBa> to vpImage<unsigned char>
    ////////////////////////////////////
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");

    /* Read the color image */

    // Load a color image from the disk
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(Ic, filename) ;
    vpImage<unsigned char> R,G,B,a;
    vpImageConvert::split(Ic, &R,NULL,&B);
    double begintime  = vpTime::measureTimeMs();
    for(int i=0; i<1000;i++){
      vpImageConvert::split(Ic, &R,NULL,&B);
    }
    double endtime = vpTime::measureTimeMs();

    std::cout<<"Time for 1000 split (ms): "<< endtime - begintime <<std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_RChannel.pgm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(R, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    filename =  vpIoTools::createFilePath(opath, "Klimt_BChannel.pgm");
    /* Save the the current image */
    vpCTRACE << "Write " << filename << std::endl;
    vpImageIo::write(B, filename) ;
    vpCTRACE<< "Convert result in "<<std::endl<< filename << std::endl;

    ////////////////////////////////////
    // Merge 4 vpImage<unsigned char> (RGBa) to vpImage<vpRGBa>
    ////////////////////////////////////
    vpImageConvert::split(Ic, &R, &G, &B, &a);
    begintime  = vpTime::measureTimeMs();
    vpImage<vpRGBa> I_merge;
    for(int i=0; i<1000; i++){
      vpImageConvert::merge(&R, &G, &B, &a, I_merge);
    }
    endtime = vpTime::measureTimeMs();

    std::cout<<"Time for 1000 merge (ms): "<< endtime - begintime <<std::endl;

    filename =  vpIoTools::createFilePath(opath, "Klimt_merge.ppm");
    /* Save the the current image */
    vpImageIo::write(I_merge, filename) ;

    ////////////////////////////////////
    // Convert a vpImage<vpRGBa> in RGB color space to a vpImage<vpRGBa> in HSV color
    ////////////////////////////////////
    unsigned int size = Ic.getWidth()*Ic.getHeight();
    unsigned int w = Ic.getWidth(), h = Ic.getHeight();
    unsigned char *hue = new unsigned char[size];
    unsigned char *saturation = new unsigned char[size];
    unsigned char *value = new unsigned char[size];

    vpImageConvert::RGBaToHSV((unsigned char *) Ic.bitmap, hue, saturation, value, size);
    vpImage<unsigned char> I_hue(hue, h, w);
    vpImage<unsigned char> I_saturation(saturation, h, w);
    vpImage<unsigned char> I_value(value, h, w);
    vpImage<vpRGBa> I_HSV;
    vpImageConvert::merge(&I_hue, &I_saturation, &I_value, NULL, I_HSV);

    filename =  vpIoTools::createFilePath(opath, "Klimt_HSV.ppm");
    /* Save the the current image */
    vpImageIo::write(I_HSV, filename);

    //Check the conversion RGBa <==> HSV
    double *hue2 = new double[size];
    double *saturation2 = new double[size];
    double *value2 = new double[size];
    vpImageConvert::RGBaToHSV((unsigned char *) Ic.bitmap, hue2, saturation2, value2, size);

    unsigned char *rgba = new unsigned char[size*4];
    vpImageConvert::HSVToRGBa(hue2, saturation2, value2, rgba, size);

    if(hue2 != NULL) {
      delete[] hue2;
      hue2 = NULL;
    }

    if(saturation2 != NULL) {
      delete[] saturation2;
      saturation2 = NULL;
    }

    if(value2 != NULL) {
      delete[] value2;
      value2 = NULL;
    }

    vpImage<vpRGBa> I_HSV2RGBa((vpRGBa *) rgba, h, w);
    filename =  vpIoTools::createFilePath(opath, "Klimt_HSV2RGBa.ppm");
    /* Save the the current image */
    vpImageIo::write(I_HSV2RGBa, filename);

    for(unsigned int i = 0; i < Ic.getHeight(); i++) {
      for(unsigned int j = 0; j < Ic.getWidth(); j++) {
        if(Ic[i][j].R != I_HSV2RGBa[i][j].R ||
           Ic[i][j].G != I_HSV2RGBa[i][j].G ||
           Ic[i][j].B != I_HSV2RGBa[i][j].B) {
          std::cerr << "Ic[i][j].R=" << static_cast<unsigned>(Ic[i][j].R)
              << " ; I_HSV2RGBa[i][j].R=" << static_cast<unsigned>(I_HSV2RGBa[i][j].R) << std::endl;
          std::cerr << "Ic[i][j].G=" << static_cast<unsigned>(Ic[i][j].G)
              << " ; I_HSV2RGBa[i][j].G=" << static_cast<unsigned>(I_HSV2RGBa[i][j].G) << std::endl;
          std::cerr << "Ic[i][j].B=" << static_cast<unsigned>(Ic[i][j].B)
              << " ; I_HSV2RGBa[i][j].B=" << static_cast<unsigned>(I_HSV2RGBa[i][j].B) << std::endl;
          throw vpException(vpException::fatalError, "Problem with conversion between RGB <==> HSV");
        }
      }
    }

    ////////////////////////////////////
    // Test construction of vpImage from an array with copyData==true
    ////////////////////////////////////
    unsigned char *rgba2 = new unsigned char[size*4];
    memset(rgba2, 127, size*4);
    vpImage<vpRGBa> I_copyData((vpRGBa *) rgba2, h, w, true);

    // Delete the array
    delete[] rgba2;

    filename =  vpIoTools::createFilePath(opath, "I_copyData.ppm");
    /* Save the the current image */
    vpImageIo::write(I_copyData, filename);

    if(I_copyData.getSize() > 0) {
      I_copyData[0][0].R = 10;
    }


    //Benchmark and test RGBa / RGB / cv::Mat to Grayscale conversion
    {
      //RGBa to Grayscale
      vpImage<vpRGBa> I_color;
      filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
      vpImageIo::read(I_color, filename);

      vpImage<unsigned char> I_gray_sse(I_color.getHeight(), I_color.getWidth());
      vpImage<unsigned char> I_gray_regular(I_color.getHeight(), I_color.getWidth());
      unsigned char value_sse = 0, value_regular = 0;

      bool fastConversion = true; //Say explicitly to use SSE code if available

      double t_sse = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        vpImageConvert::convert(I_color, I_gray_sse, fastConversion);
        value_sse += I_gray_sse[0][0];
      }
      t_sse = vpTime::measureTimeMs() - t_sse;

      double t_regular = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        computeRegularRGBaToGrayscale((unsigned char *) I_color.bitmap, I_gray_regular.bitmap, I_color.getSize());
        value_regular += I_gray_regular[0][0];
      }
      t_regular = vpTime::measureTimeMs() - t_regular;

      //Compute the error between the SSE and regular version
      double rmse_error = 0.0;
      for(unsigned int i = 0; i < I_color.getHeight(); i++) {
        for(unsigned int j = 0; j < I_color.getWidth(); j++) {
          rmse_error += (I_gray_sse[i][j] - I_gray_regular[i][j]) * (I_gray_sse[i][j] - I_gray_regular[i][j]);
        }
      }

      std::cout << "\nRGBa to Grayscale" << std::endl;
      std::cout << "t_regular (" << nbIterations << " iterations)=" << t_regular << " ms"
                << " ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;
      std::cout << "Speed-up=" << (t_regular/t_sse) << "X" << std::endl;
      std::cout << "RMSE error between SSE and regular version: " << (std::sqrt(rmse_error/I_color.getSize())) << std::endl;

      //To prevent the iteration loop to not be optimized?
      std::cout << "value_sse=" << static_cast<unsigned>(value_sse)
          << " ; value_regular=" << static_cast<unsigned>(value_regular) << std::endl;

      filename =  vpIoTools::createFilePath(opath, "I_rgba2gray_sse.pgm");
      vpImageIo::write(I_gray_sse, filename);

      filename =  vpIoTools::createFilePath(opath, "I_rgba2gray_regular.pgm");
      vpImageIo::write(I_gray_regular, filename);


      //RGB to Grayscale conversion
      unsigned char *rgb_array = new unsigned char[I_color.getSize() * 3];
      vpImageConvert::RGBaToRGB((unsigned char *) I_color.bitmap, rgb_array, I_color.getSize());

      value_sse = 0;
      value_regular = 0;

      unsigned char *rgb2gray_array_sse = new unsigned char[I_color.getSize()];
      t_sse = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        vpImageConvert::RGBToGrey(rgb_array, rgb2gray_array_sse, I_color.getWidth(), I_color.getHeight(), false, fastConversion);
        value_sse += rgb2gray_array_sse[0];
      }
      t_sse = vpTime::measureTimeMs() - t_sse;

      unsigned char *rgb2gray_array_regular = new unsigned char[I_color.getSize()];
      t_regular = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        computeRegularRGBToGrayscale(rgb_array, rgb2gray_array_regular, I_color.getSize());
        value_regular += rgb2gray_array_regular[0];
      }
      t_regular = vpTime::measureTimeMs() - t_regular;

      vpImage<unsigned char> I_gray2rgba_sse(rgb2gray_array_sse, I_color.getHeight(), I_color.getWidth(), false);
      vpImage<unsigned char> I_gray2rgba_regular(rgb2gray_array_regular, I_color.getHeight(), I_color.getWidth(), false);

      //Compute the error between the SSE and regular version
      rmse_error = 0.0;
      for(unsigned int i = 0; i < I_color.getHeight(); i++) {
        for(unsigned int j = 0; j < I_color.getWidth(); j++) {
          rmse_error += (I_gray2rgba_sse[i][j] - I_gray2rgba_regular[i][j]) * (I_gray2rgba_sse[i][j] - I_gray2rgba_regular[i][j]);
        }
      }

      std::cout << "\nRGB to Grayscale" << std::endl;
      std::cout << "t_regular (" << nbIterations << " iterations)=" << t_regular << " ms"
                << " ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;
      std::cout << "Speed-up=" << (t_regular/t_sse) << "X" << std::endl;
      std::cout << "RMSE error between SSE and regular version: " << (std::sqrt(rmse_error/I_color.getSize())) << std::endl;

      //To prevent the iteration loop to not be optimized?
      std::cout << "value_sse=" << static_cast<unsigned>(value_sse)
          << " ; value_regular=" << static_cast<unsigned>(value_regular) << std::endl;

      filename =  vpIoTools::createFilePath(opath, "I_rgb2gray_sse.pgm");
      vpImageIo::write(I_gray2rgba_sse, filename);

      filename =  vpIoTools::createFilePath(opath, "I_rgb2gray_regular.pgm");
      vpImageIo::write(I_gray2rgba_regular, filename);


#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
      //BGR cv::Mat to Grayscale
      filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
      cv::Mat colorMat = cv::imread(filename);
      std::cout << "colorMat=" << colorMat.cols << "x" << colorMat.rows << std::endl;

      vpImage<unsigned char> I_mat2gray_sse, I_mat2gray_regular;
      value_sse = 0;
      value_regular = 0;

      t_sse = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        vpImageConvert::convert(colorMat, I_mat2gray_sse, false, fastConversion);
        value_sse += I_mat2gray_sse[0][0];
      }
      t_sse = vpTime::measureTimeMs() - t_sse;

      t_regular = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        computeRegularBGRToGrayscale(colorMat, I_mat2gray_regular);
        value_regular += I_mat2gray_sse[0][0];
      }
      t_regular = vpTime::measureTimeMs() - t_regular;

      //Compute the error between the SSE and regular version
      rmse_error = 0.0;
      for(unsigned int i = 0; i < I_color.getHeight(); i++) {
        for(unsigned int j = 0; j < I_color.getWidth(); j++) {
          rmse_error += (I_mat2gray_sse[i][j] - I_mat2gray_regular[i][j]) * (I_mat2gray_sse[i][j] - I_mat2gray_regular[i][j]);
        }
      }

      std::cout << "\nBGR Mat to Grayscale" << std::endl;
      std::cout << "t_regular (" << nbIterations << " iterations)=" << t_regular << " ms"
                << " ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;
      std::cout << "Speed-up=" << (t_regular/t_sse) << "X" << std::endl;
      std::cout << "RMSE error between SSE and regular version: " << (std::sqrt(rmse_error/I_color.getSize())) << std::endl;

      //To prevent the iteration loop to not be optimized?
      std::cout << "value_sse=" << static_cast<unsigned>(value_sse)
          << " ; value_regular=" << static_cast<unsigned>(value_regular) << std::endl;

      filename =  vpIoTools::createFilePath(opath, "I_mat2gray_sse.pgm");
      vpImageIo::write(I_mat2gray_sse, filename);

      filename =  vpIoTools::createFilePath(opath, "I_mat2gray_regular.pgm");
      vpImageIo::write(I_mat2gray_regular, filename);


      //BGR cv::Mat to Grayscale cv::Mat
      cv::Mat grayscaleMat(colorMat.size(), CV_8U);
      unsigned char value_mat = 0;

      double t_opencv = vpTime::measureTimeMs();
      for(int iteration = 0; iteration < nbIterations; iteration++) {
        cv::cvtColor(colorMat, grayscaleMat, cv::COLOR_BGR2GRAY);
        value_mat += grayscaleMat.ptr<uchar>(0)[0];
      }
      t_opencv = vpTime::measureTimeMs() - t_opencv;

      std::cout << "\nBGR Mat to Grayscale Mat" << std::endl;
      std::cout << "t_opencv (" << nbIterations << " iterations)=" << t_opencv << " ms"
                << " ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;
      std::cout << "Speed-up=" << (t_opencv/t_sse) << "X" << std::endl;
      std::cout << "value_mat=" << static_cast<unsigned>(value_mat) << std::endl;

      vpImage<unsigned char> I_grayscale_mat;
      vpImageConvert::convert(grayscaleMat, I_grayscale_mat);
      filename =  vpIoTools::createFilePath(opath, "grayscaleMat.pgm");
      vpImageIo::write(I_grayscale_mat, filename);


      //Test RGB to Grayscale + Flip
      unsigned char *rgb2gray_flip_array_sse = new unsigned char[I_color.getSize()];
      vpImageConvert::RGBToGrey(rgb_array, rgb2gray_flip_array_sse, I_color.getWidth(), I_color.getHeight(), true);
      vpImage<unsigned char> I_rgb2gray_flip_sse(rgb2gray_flip_array_sse, I_color.getHeight(), I_color.getWidth());

      filename =  vpIoTools::createFilePath(opath, "I_rgb2gray_flip_sse.pgm");
      vpImageIo::write(I_rgb2gray_flip_sse, filename);


      //Test BGR to Grayscale + Flip
      unsigned char *bgr2gray_flip_array_sse = new unsigned char[I_color.getSize()];
      vpImage<unsigned char> I_bgr2gray_flip_sse(bgr2gray_flip_array_sse, I_color.getHeight(), I_color.getWidth());
      vpImageConvert::convert(colorMat, I_bgr2gray_flip_sse, true, fastConversion);

      filename =  vpIoTools::createFilePath(opath, "I_bgr2gray_flip_sse.pgm");
      vpImageIo::write(I_bgr2gray_flip_sse, filename);


      //Test RGB to Grayscale + Flip + Crop
      cv::Rect rect_roi(11, 17, 347, 449);
      cv::Mat colorMat_crop = colorMat(rect_roi);
      cv::Mat colorMat_crop_continous = colorMat(rect_roi).clone();
      std::cout << "colorMat_crop: " << colorMat_crop.cols << "x" << colorMat_crop.rows <<
                   " is continuous? " << colorMat_crop.isContinuous() << std::endl;
      std::cout << "colorMat_crop_continous: " << colorMat_crop_continous.cols
                << "x" << colorMat_crop_continous.rows << " is continuous? "
                << colorMat_crop_continous.isContinuous() << std::endl;

      vpImage<vpRGBa> I_color_crop( (unsigned int) (rect_roi.height-rect_roi.y), (unsigned int) (rect_roi.width-rect_roi.x) );
      for(unsigned int i = (unsigned int) rect_roi.y; i < (unsigned int) rect_roi.height; i++) {
        for(unsigned int j = (unsigned int) rect_roi.x; j < (unsigned int) rect_roi.width; j++) {
          I_color_crop[(unsigned int) ((int)i-rect_roi.y)][(unsigned int) ((int)j-rect_roi.x)] = I_color[i][j];
        }
      }

      unsigned char *rgb_array_crop = new unsigned char[I_color_crop.getSize()*3];
      vpImageConvert::RGBaToRGB((unsigned char *) I_color_crop.bitmap, rgb_array_crop, I_color_crop.getSize());

      unsigned char *rgb2gray_flip_crop_array_sse = new unsigned char[I_color_crop.getSize()];
      vpImageConvert::RGBToGrey(rgb_array_crop, rgb2gray_flip_crop_array_sse, I_color_crop.getWidth(),
                                I_color_crop.getHeight(), true, fastConversion);
      vpImage<unsigned char> I_rgb2gray_flip_crop_sse(rgb2gray_flip_crop_array_sse, I_color_crop.getHeight(),
                                                      I_color_crop.getWidth());

      filename =  vpIoTools::createFilePath(opath, "I_rgb2gray_flip_crop_sse.pgm");
      vpImageIo::write(I_rgb2gray_flip_crop_sse, filename);


      //Test BGR to Grayscale + Flip + Crop
      vpImage<unsigned char> I_bgr2gray_flip_crop_sse(I_color_crop.getHeight(), I_color_crop.getWidth());
      vpImageConvert::convert(colorMat_crop_continous, I_bgr2gray_flip_crop_sse, true, fastConversion);

      filename =  vpIoTools::createFilePath(opath, "I_bgr2gray_flip_crop_sse.pgm");
      vpImageIo::write(I_bgr2gray_flip_crop_sse, filename);


      //Test BGR to Grayscale + Flip + Crop + No continuous Mat
      vpImage<unsigned char> I_bgr2gray_flip_crop_no_continuous_sse(I_color_crop.getHeight(),
                                                                    I_color_crop.getWidth());
      vpImageConvert::convert(colorMat_crop, I_bgr2gray_flip_crop_no_continuous_sse, true, fastConversion);

      filename =  vpIoTools::createFilePath(opath, "I_bgr2gray_flip_crop_no_continuous_sse.pgm");
      vpImageIo::write(I_bgr2gray_flip_crop_no_continuous_sse, filename);
#endif

      //Delete
      delete[] rgb_array;
    }



    //Compare SSE and AVX
//    {
//      vpImage<vpRGBa> I_color;
//      filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.ppm");
//      vpImageIo::read(I_color, filename);

//      unsigned char *rgb_array = new unsigned char[I_color.getSize()*3];
//      vpImageConvert::RGBaToRGB((unsigned char *) I_color.bitmap, rgb_array, I_color.getSize());

//      unsigned char *grey_SSE = new unsigned char[I_color.getSize()];
//      unsigned char *grey_AVX = new unsigned char[I_color.getSize()];

//      test_SSE(rgb_array, grey_SSE, I_color.getWidth(), I_color.getHeight());
//      test_AVX(rgb_array, grey_AVX, I_color.getWidth(), I_color.getHeight());

//      for(int i = 0; i < 32; i++) {
//        std::cout << "grey_SSE[" << i << "]=" << static_cast<unsigned>(grey_SSE[i]) << " ; ";
//        std::cout << "grey_AVX[" << i << "]=" << static_cast<unsigned>(grey_AVX[i]) << std::endl;
//      }
//    }


    return 0;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
