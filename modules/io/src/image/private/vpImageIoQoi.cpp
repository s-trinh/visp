/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Support of the QOI image format.
 *
 *****************************************************************************/

/*!
  \file vpImageIoQoi.cpp
  \brief Support of the QOI image format.
*/

#include "vpImageIoBackend.h"
#include <visp3/core/vpImageConvert.h>

#define QOI_IMPLEMENTATION
#include "qoi.h"

void vp_readQOI(vpImage<unsigned char> &I, const std::string &filename)
{
  qoi_desc desc;
  void *pixels = qoi_read(filename.c_str(), &desc, 0);

  if (desc.channels == 1) {
    const bool copyData = true;
    I.init(reinterpret_cast<unsigned char * const>(pixels), desc.height, desc.width, copyData);
    free(pixels);
  } else if (desc.channels == 3) {
    I.init(desc.height, desc.width);
    vpImageConvert::RGBToGrey(reinterpret_cast<unsigned char *>(pixels), I.bitmap, I.getSize());
    free(pixels);
  } else if (desc.channels == 4) {
    I.init(desc.height, desc.width);
    vpImageConvert::RGBaToGrey(reinterpret_cast<unsigned char *>(pixels), I.bitmap, I.getSize());
    free(pixels);
  } else {
    free(pixels);
    std::ostringstream oss;
    oss << "Unsupported number of channels: " << desc.channels;
    throw vpException(vpImageException::ioError, oss.str());
  }
}

void vp_readQOI(vpImage<vpRGBa> &I, const std::string &filename)
{
  qoi_desc desc;
  void *pixels = qoi_read(filename.c_str(), &desc, 0);

  if (desc.channels == 4) {
    const bool copyData = true;
    I.init(reinterpret_cast<vpRGBa * const>(pixels), desc.height, desc.width, copyData);
    free(pixels);
  } else if (desc.channels == 3) {
    I.init(desc.height, desc.width);
    vpImageConvert::RGBToRGBa(reinterpret_cast<unsigned char *>(pixels), reinterpret_cast<unsigned char *>(I.bitmap), I.getSize());
    free(pixels);
  } else if (desc.channels == 1) {
    I.init(desc.height, desc.width);
    vpImageConvert::GreyToRGBa(reinterpret_cast<unsigned char *>(pixels), reinterpret_cast<unsigned char *> (I.bitmap), I.getSize());
    free(pixels);
  } else {
    free(pixels);
    std::ostringstream oss;
    oss << "Unsupported number of channels: " << desc.channels;
    throw vpException(vpImageException::ioError, oss.str());
  }
}

void vp_writeQOI(const vpImage<unsigned char> &I, const std::string &filename)
{
  qoi_desc desc;
  desc.width = I.getWidth();
  desc.height = I.getHeight();
  desc.channels = 1;
  desc.colorspace = QOI_LINEAR;
  int encoded = qoi_write(filename.c_str(), reinterpret_cast<const void *>(I.bitmap), &desc);
}

void vp_writeQOI(const vpImage<vpRGBa> &I, const std::string &filename)
{
  qoi_desc desc;
  desc.width = I.getWidth();
  desc.height = I.getHeight();
  desc.channels = 4;
  desc.colorspace = QOI_SRGB;
  int encoded = qoi_write(filename.c_str(), reinterpret_cast<const void *>(I.bitmap), &desc);
}
