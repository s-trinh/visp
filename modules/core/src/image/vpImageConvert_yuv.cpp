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
 */

/*!
  \file vpImageConvert_yuv.cpp
  \brief Various yuv formats to RGB and RGBa conversion.
*/


#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

namespace
{
void vpSAT(int &c)
{
  if (c < 0) {
    c = 0;
  }
  else {
    c = 255;
  }
}
};

/*!
  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...) to RGB32.
  Destination rgba memory area has to be allocated before.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuyv : Pointer to the bitmap containing the YUYV 4:2:2 data.
  \param[out] rgba : Pointer to the RGB32 bitmap that should be allocated with a size of \e width * \e height * 4.
  \param[in] width, height : Image size.

  \sa YUV422ToRGBa()
*/
void vpImageConvert::YUYVToRGBa(unsigned char *yuyv, unsigned char *rgba, unsigned int width, unsigned int height)
{
  unsigned char *s;
  unsigned char *d;
  int w, h;
  int r, g, b, cr, cg, cb, y1, y2;

  h = static_cast<int>(height);
  w = static_cast<int>(width);
  s = yuyv;
  d = rgba;
  while (h--) {
    int c = w / 2;
    while (c--) {
      y1 = *s;
      ++s;
      cb = ((*s - 128) * 454) / 256;
      cg = (*s - 128) * 88;
      ++s;
      y2 = *s;
      ++s;
      cr = ((*s - 128) * 359) / 256;
      cg = (cg + ((*s - 128) * 183)) / 256;
      ++s;

      r = y1 + cr;
      b = y1 + cb;
      g = y1 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
      *d++ = vpRGBa::alpha_default;

      r = y2 + cr;
      b = y2 + cb;
      g = y2 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
      *d++ = vpRGBa::alpha_default;
    }
  }
}

/*!
  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...)
  to RGB24. Destination rgb memory area has to be allocated before.

  \param[in] yuyv : Pointer to the bitmap containing the YUYV 4:2:2 data.
  \param[out] rgb : Pointer to the RGB32 bitmap that should be allocated with a size of \e width * \e height * 3.
  \param[in] width, height : Image size.

  \sa YUV422ToRGB()
*/
void vpImageConvert::YUYVToRGB(unsigned char *yuyv, unsigned char *rgb, unsigned int width, unsigned int height)
{
  unsigned char *s;
  unsigned char *d;
  int h, w;
  int r, g, b, cr, cg, cb, y1, y2;

  h = static_cast<int>(height);
  w = static_cast<int>(width);
  s = yuyv;
  d = rgb;
  while (h--) {
    int c = w / 2;
    while (c--) {
      y1 = *s;
      ++s;
      cb = ((*s - 128) * 454) / 256;
      cg = (*s - 128) * 88;
      ++s;
      y2 = *s;
      ++s;
      cr = ((*s - 128) * 359) / 256;
      cg = (cg + ((*s - 128) * 183)) / 256;
      ++s;

      r = y1 + cr;
      b = y1 + cb;
      g = y1 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);

      r = y2 + cr;
      b = y2 + cb;
      g = y2 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
    }
  }
}

/*!
  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...)
  to grey. Destination grey memory area has to be allocated before.

  \param[in] yuyv : Pointer to the bitmap containing the YUYV 4:2:2 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should be allocated with a size of width * height.
  \param[in] size : Image size corresponding to width * height.

  \sa YUV422ToGrey()
*/
void vpImageConvert::YUYVToGrey(unsigned char *yuyv, unsigned char *grey, unsigned int size)
{
  unsigned int i = 0, j = 0;
  const unsigned int doubleSize = size * 2;
  while (j < doubleSize) {
    grey[i++] = yuyv[j];
    grey[i++] = yuyv[j + 2];
    j += 4;
  }
}

/*!
  Convert YUV 4:1:1 (u y1 y2 v y3 y4) images into RGBa images. The alpha
  component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:1:1 data.
  \param[out] rgba : Pointer to the RGBA 32-bits bitmap that should be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to width * height.

*/
void vpImageConvert::YUV411ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size)
{
  for (unsigned int i = size / 4; i; --i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y0 = *yuv;
    ++yuv;
    int Y1 = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int Y2 = *yuv;
    ++yuv;
    int Y3 = *yuv;
    ++yuv;
    int UV = -U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    int R = Y0 + V2;
    vpSAT(R);

    int G = Y0 + UV;
    vpSAT(G);

    int B = Y0 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;

    //---
    R = Y1 + V2;
    vpSAT(R);

    G = Y1 + UV;
    vpSAT(G);

    B = Y1 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;

    //---
    R = Y2 + V2;
    vpSAT(R);

    G = Y2 + UV;
    vpSAT(G);

    B = Y2 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;

    //---
    R = Y3 + V2;
    vpSAT(R);

    G = Y3 + UV;
    vpSAT(G);

    B = Y3 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;
  }
}

/*!
  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into RGBa images.
  Destination rgba memory area has to be allocated before.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:2 data.
  \param[out] rgba : Pointer to the RGBA 32-bits bitmap that should be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to width * height.

  \sa YUYVToRGBa()
*/
void vpImageConvert::YUV422ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size)
{
  for (unsigned int i = size / 2; i; --i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y0 = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int Y1 = *yuv;
    ++yuv;
    int UV = -U - V;

    //---
    int R = Y0 + V2;
    vpSAT(R);

    int G = Y0 + UV;
    vpSAT(G);

    int B = Y0 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;

    //---
    R = Y1 + V2;
    vpSAT(R);

    G = Y1 + UV;
    vpSAT(G);

    B = Y1 + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;
  }
}

/*!
  Convert YUV 4:1:1 (u y1 y2 v y3 y4) into a grey image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:1:1 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should be allocated with a size of width * height.
  \param[in] size : Image size corresponding to width * height.

*/
void vpImageConvert::YUV411ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size)
{
  unsigned int i = 0, j = 0;
  const unsigned int iterLimit = (size * 3) / 2;
  while (j < iterLimit) {
    grey[i] = yuv[j + 1];
    grey[i + 1] = yuv[j + 2];
    grey[i + 2] = yuv[j + 4];
    grey[i + 3] = yuv[j + 5];

    i += 4;

    j += 6;
  }
}

/*!
  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into RGB images.
  Destination rgb memory area has to be allocated before.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:2 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] size : Image size corresponding to width * height.

  \sa YUYVToRGB()
*/
void vpImageConvert::YUV422ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size)
{
  for (unsigned int i = size / 2; i; --i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y0 = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int Y1 = *yuv;
    ++yuv;
    int UV = -U - V;

    //---
    int R = Y0 + V2;
    vpSAT(R);

    int G = Y0 + UV;
    vpSAT(G);

    int B = Y0 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);

    //---
    R = Y1 + V2;
    vpSAT(R);

    G = Y1 + UV;
    vpSAT(G);

    B = Y1 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);
  }
}

/*!
  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into a grey image.
  Destination grey memory area has to be allocated before.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:2 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should be allocated with a size of width * height.
  \param[in] size : Image size corresponding to width * height.

  \sa YUYVToGrey()
*/
void vpImageConvert::YUV422ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size)
{
  unsigned int i = 0, j = 0;
  const unsigned int doubleSize = size * 2;

  while (j < doubleSize) {
    grey[i++] = yuv[j + 1];
    grey[i++] = yuv[j + 3];
    j += 4;
  }
}

/*!
  Convert YUV 4:1:1 (u y1 y2 v y3 y4) into a RGB 24bits image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:1:1 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] size : Image size corresponding to width * height.

*/
void vpImageConvert::YUV411ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size)
{
  for (unsigned int i = size / 4; i; --i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y0 = *yuv;
    ++yuv;
    int Y1 = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int Y2 = *yuv;
    ++yuv;
    int Y3 = *yuv;
    ++yuv;
    int UV = -U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    int R = Y0 + V2;
    vpSAT(R);

    int G = Y0 + UV;
    vpSAT(G);

    int B = Y0 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);

    //---
    R = Y1 + V2;
    vpSAT(R);

    G = Y1 + UV;
    vpSAT(G);

    B = Y1 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);

    //---
    R = Y2 + V2;
    vpSAT(R);

    G = Y2 + UV;
    vpSAT(G);

    B = Y2 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);

    //---
    R = Y3 + V2;
    vpSAT(R);

    G = Y3 + UV;
    vpSAT(G);

    B = Y3 + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);
  }
}

/*!
  Convert YUV 4:2:0 [Y(NxM), U(N/2xM/2), V(N/2xM/2)] image into a RGBa image.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:0 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.

*/
void vpImageConvert::YUV420ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3;
  unsigned int size = width * height;
  unsigned char *iU = yuv + size;
  unsigned char *iV = yuv + ((5 * size) / 4);
  const unsigned int halfHeight = height / 2, halfWidth = width / 2;
  for (unsigned int i = 0; i < halfHeight; ++i) {
    for (unsigned int j = 0; j < halfWidth; ++j) {
      U = static_cast<int>(((*iU) - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>(((*iV) - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      yuv = yuv + (width - 1);
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = (yuv - width) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = (rgba + (4 * width)) - 7;

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = (rgba - (4 * width)) + 1;
    }
    yuv += width;
    rgba += 4 * width;
  }
}

/*!
  Convert YUV 4:2:0 [Y(NxM), U(N/2xM/2), V(N/2xM/2)] image into a RGB image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:0 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] width, height : Image size.

*/
void vpImageConvert::YUV420ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3;
  unsigned int size = width * height;
  unsigned char *iU = yuv + size;
  unsigned char *iV = yuv + ((5 * size) / 4);
  const unsigned int halfHeight = height / 2, halfWidth = width / 2;
  for (unsigned int i = 0; i < halfHeight; ++i) {
    for (unsigned int j = 0; j < halfWidth; ++j) {
      U = static_cast<int>(((*iU) - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>(((*iV) - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      yuv = yuv + (width - 1);
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = (yuv - width) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = rgb + ((3 * width) - 5);

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = (rgb - (3 * width)) + 1;
    }
    yuv += width;
    rgb += 3 * width;
  }
}

/*!
  Convert YUV 4:2:0 [Y(NxM), U(N/2xM/2), V(N/2xM/2)] image into a grey image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:2:0 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should be allocated with a size of width * height.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YUV420ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    *grey++ = *yuv;
    ++yuv;
  }
}

/*!
  Convert YUV 4:4:4 (u y v) image into a RGBa image.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:4:4 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] size : Image size corresponding to image width * height.
*/
void vpImageConvert::YUV444ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int UV = -U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    int R = Y + V2;
    vpSAT(R);

    int G = Y + UV;
    vpSAT(G);

    int B = Y + U5;
    vpSAT(B);

    *rgba++ = static_cast<unsigned char>(R);
    *rgba++ = static_cast<unsigned char>(G);
    *rgba++ = static_cast<unsigned char>(B);
    *rgba++ = vpRGBa::alpha_default;
  }
}

/*!
  Convert YUV 4:4:4 (u y v) image into RGB image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:4:4 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] size: Image size corresponding to image width * height.
*/
void vpImageConvert::YUV444ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    int U = static_cast<int>((*yuv - 128) * 0.354);
    ++yuv;
    int U5 = 5 * U;
    int Y = *yuv;
    ++yuv;
    int V = static_cast<int>((*yuv - 128) * 0.707);
    ++yuv;
    int V2 = 2 * V;
    int UV = -U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    int R = Y + V2;
    vpSAT(R);

    int G = Y + UV;
    vpSAT(G);

    int B = Y + U5;
    vpSAT(B);

    *rgb++ = static_cast<unsigned char>(R);
    *rgb++ = static_cast<unsigned char>(G);
    *rgb++ = static_cast<unsigned char>(B);
  }
}

/*!
  Convert YUV 4:4:4 (u y v) image into a grey image.

  \param[in] yuv : Pointer to the bitmap containing the YUV 4:4:4 data.
  \param[out] grey : Pointer to the 8-bits grey bitmap that should be allocated with a size of width * height.
  \param[in] size: Image size corresponding to image width * height.
*/
void vpImageConvert::YUV444ToGrey(unsigned char *yuv, unsigned char *grey, unsigned int size)
{
  ++yuv;
  for (unsigned int i = 0; i < size; ++i) {
    *grey++ = *yuv;
    yuv = yuv + 3;
  }
}

/*!
  Convert YV 1:2 [Y(NxM), V(N/2xM/2), U(N/2xM/2)] image into RGBa image.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YV 1:2 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.

*/
void vpImageConvert::YV12ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3;
  unsigned int size = width * height;
  unsigned char *iV = yuv + size;
  unsigned char *iU = yuv + ((5 * size) / 4);
  const unsigned int halfHeight = height / 2, halfWidth = width / 2;
  for (unsigned int i = 0; i < halfHeight; ++i) {
    for (unsigned int j = 0; j < halfWidth; ++j) {
      U = static_cast<int>(((*iU) - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>(((*iV) - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      yuv = yuv + (width - 1);
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = (yuv - width) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = 0;
      rgba = rgba + ((4 * width) - 7);

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = (rgba - (4 * width)) + 1;
    }
    yuv += width;
    rgba += 4 * width;
  }
}

/*!
  Convert YV12 [Y(NxM), V(N/2xM/2), U(N/2xM/2)] image into RGB image.

  \param[in] yuv : Pointer to the bitmap containing the YV 1:2 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] width, height : Image size.

*/
void vpImageConvert::YV12ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int height, unsigned int width)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3;
  unsigned int size = width * height;
  unsigned char *iV = yuv + size;
  unsigned char *iU = yuv + ((5 * size) / 4);
  const unsigned int halfHeight = height / 2, halfWidth = width / 2;
  for (unsigned int i = 0; i < halfHeight; ++i) {
    for (unsigned int j = 0; j < halfWidth; ++j) {
      U = static_cast<int>(((*iU) - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>(((*iV) - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      yuv = yuv + (width - 1);
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = (yuv - width) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = rgb + ((3 * width) - 5);

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = (rgb - (3 * width)) + 1;
    }
    yuv += width;
    rgb += 3 * width;
  }
}

/*!
  Convert YVU 9 [Y(NxM), V(N/4xM/4), U(N/4xM/4)] image into a RGBa image.

  The alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] yuv : Pointer to the bitmap containing the YVU 9 data.
  \param[out] rgba : Pointer to the 32-bits RGBA bitmap that should be allocated with a size of width * height * 4.
  \param[in] width, height : Image size.

*/
void vpImageConvert::YVU9ToRGBa(unsigned char *yuv, unsigned char *rgba, unsigned int width, unsigned int height)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10, Y11, Y12, Y13, Y14, Y15;
  unsigned int size = width * height;
  unsigned char *iV = yuv + size;
  unsigned char *iU = yuv + ((17 * size) / 16);
  const unsigned int quarterHeight = height / 4, quarterWidth = width / 4;
  for (unsigned int i = 0; i < quarterHeight; ++i) {
    for (unsigned int j = 0; j < quarterWidth; ++j) {
      U = static_cast<int>(((*iU) - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>(((*iV) - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      ++yuv;
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = yuv + (width - 3);
      Y4 = *yuv;
      ++yuv;
      Y5 = *yuv;
      ++yuv;
      Y6 = *yuv;
      ++yuv;
      Y7 = *yuv;
      yuv = yuv + (width - 3);
      Y8 = *yuv;
      ++yuv;
      Y9 = *yuv;
      ++yuv;
      Y10 = *yuv;
      ++yuv;
      Y11 = *yuv;
      yuv = yuv + (width - 3);
      Y12 = *yuv;
      ++yuv;
      Y13 = *yuv;
      ++yuv;
      Y14 = *yuv;
      ++yuv;
      Y15 = *yuv;
      yuv = (yuv - (3 * width)) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = rgba + ((4 * width) - 15);

      R = Y4 + V2;
      vpSAT(R);

      G = Y4 + UV;
      vpSAT(G);

      B = Y4 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y5 + V2;
      vpSAT(R);

      G = Y5 + UV;
      vpSAT(G);

      B = Y5 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y6 + V2;
      vpSAT(R);

      G = Y6 + UV;
      vpSAT(G);

      B = Y6 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y7 + V2;
      vpSAT(R);

      G = Y7 + UV;
      vpSAT(G);

      B = Y7 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = rgba + ((4 * width) - 15);

      R = Y8 + V2;
      vpSAT(R);

      G = Y8 + UV;
      vpSAT(G);

      B = Y8 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y9 + V2;
      vpSAT(R);

      G = Y9 + UV;
      vpSAT(G);

      B = Y9 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y10 + V2;
      vpSAT(R);

      G = Y10 + UV;
      vpSAT(G);

      B = Y10 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y11 + V2;
      vpSAT(R);

      G = Y11 + UV;
      vpSAT(G);

      B = Y11 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = rgba + ((4 * width) - 15);

      R = Y12 + V2;
      vpSAT(R);

      G = Y12 + UV;
      vpSAT(G);

      B = Y12 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y13 + V2;
      vpSAT(R);

      G = Y13 + UV;
      vpSAT(G);

      B = Y13 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y14 + V2;
      vpSAT(R);

      G = Y14 + UV;
      vpSAT(G);

      B = Y14 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba++ = vpRGBa::alpha_default;

      //---
      R = Y15 + V2;
      vpSAT(R);

      G = Y15 + UV;
      vpSAT(G);

      B = Y15 + U5;
      vpSAT(B);

      *rgba++ = static_cast<unsigned char>(R);
      *rgba++ = static_cast<unsigned char>(G);
      *rgba++ = static_cast<unsigned char>(B);
      *rgba = vpRGBa::alpha_default;
      rgba = (rgba - (12 * width)) + 1;
    }
    yuv += 3 * width;
    rgba += 12 * width;
  }
}

/*!
  Convert YV 1:2 [Y(NxM),  V(N/4xM/4), U(N/4xM/4)] image into RGB image.

  \param[in] yuv : Pointer to the bitmap containing the YV 1:2 data.
  \param[out] rgb : Pointer to the 24-bits RGB bitmap that should be allocated with a size of width * height * 3.
  \param[in] width, height : Image size.
*/
void vpImageConvert::YVU9ToRGB(unsigned char *yuv, unsigned char *rgb, unsigned int height, unsigned int width)
{
  int U, V, R, G, B, V2, U5, UV;
  int Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10, Y11, Y12, Y13, Y14, Y15;
  unsigned int size = width * height;
  unsigned char *iV = yuv + size;
  unsigned char *iU = yuv + ((17 * size) / 16);
  const unsigned int quarterHeight = height / 4, quarterWidth = width / 4;
  for (unsigned int i = 0; i < quarterHeight; ++i) {
    for (unsigned int j = 0; j < quarterWidth; ++j) {
      U = static_cast<int>((*iU - 128) * 0.354);
      ++iU;
      U5 = 5 * U;
      V = static_cast<int>((*iV - 128) * 0.707);
      ++iV;
      V2 = 2 * V;
      UV = -U - V;
      Y0 = *yuv;
      ++yuv;
      Y1 = *yuv;
      ++yuv;
      Y2 = *yuv;
      ++yuv;
      Y3 = *yuv;
      yuv = yuv + (width - 3);
      Y4 = *yuv;
      ++yuv;
      Y5 = *yuv;
      ++yuv;
      Y6 = *yuv;
      ++yuv;
      Y7 = *yuv;
      yuv = yuv + (width - 3);
      Y8 = *yuv;
      ++yuv;
      Y9 = *yuv;
      ++yuv;
      Y10 = *yuv;
      ++yuv;
      Y11 = *yuv;
      yuv = yuv + (width - 3);
      Y12 = *yuv;
      ++yuv;
      Y13 = *yuv;
      ++yuv;
      Y14 = *yuv;
      ++yuv;
      Y15 = *yuv;
      yuv = (yuv - (3 * width)) + 1;

      // Original equations
      // R = Y           + 1.402 V
      // G = Y - 0.344 U - 0.714 V
      // B = Y + 1.772 U
      R = Y0 + V2;
      vpSAT(R);

      G = Y0 + UV;
      vpSAT(G);

      B = Y0 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y1 + V2;
      vpSAT(R);

      G = Y1 + UV;
      vpSAT(G);

      B = Y1 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y2 + V2;
      vpSAT(R);

      G = Y2 + UV;
      vpSAT(G);

      B = Y2 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y3 + V2;
      vpSAT(R);

      G = Y3 + UV;
      vpSAT(G);

      B = Y3 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = rgb + ((3 * width) - 11);

      R = Y4 + V2;
      vpSAT(R);

      G = Y4 + UV;
      vpSAT(G);

      B = Y4 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y5 + V2;
      vpSAT(R);

      G = Y5 + UV;
      vpSAT(G);

      B = Y5 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y6 + V2;
      vpSAT(R);

      G = Y6 + UV;
      vpSAT(G);

      B = Y6 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y7 + V2;
      vpSAT(R);

      G = Y7 + UV;
      vpSAT(G);

      B = Y7 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = rgb + ((3 * width) - 11);

      R = Y8 + V2;
      vpSAT(R);

      G = Y8 + UV;
      vpSAT(G);

      B = Y8 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y9 + V2;
      vpSAT(R);

      G = Y9 + UV;
      vpSAT(G);

      B = Y9 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y10 + V2;
      vpSAT(R);

      G = Y10 + UV;
      vpSAT(G);

      B = Y10 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y11 + V2;
      vpSAT(R);

      G = Y11 + UV;
      vpSAT(G);

      B = Y11 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb = static_cast<unsigned char>(B);
      rgb = (rgb + (3 * width)) - 11;

      R = Y12 + V2;
      vpSAT(R);

      G = Y12 + UV;
      vpSAT(G);

      B = Y12 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y13 + V2;
      vpSAT(R);

      G = Y13 + UV;
      vpSAT(G);

      B = Y13 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y14 + V2;
      vpSAT(R);

      G = Y14 + UV;
      vpSAT(G);

      B = Y14 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);

      //---
      R = Y15 + V2;
      vpSAT(R);

      G = Y15 + UV;
      vpSAT(G);

      B = Y15 + U5;
      vpSAT(B);

      *rgb++ = static_cast<unsigned char>(R);
      *rgb++ = static_cast<unsigned char>(G);
      *rgb++ = static_cast<unsigned char>(B);
      rgb = (rgb - (9 * width)) + 1;
    }
    yuv += 3 * width;
    rgb += 9 * width;
  }
}
