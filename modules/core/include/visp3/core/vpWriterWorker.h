/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Virtual base class providing different methods to be adapted.
 */

/*!
  \file vpWriterWorker.h
  \brief Virtual base class providing different methods to be adapted.
*/

#ifndef _vpWriterWorker_h_
#define _vpWriterWorker_h_

#include <visp3/core/vpConfig.h>

/*!
  \class vpWriterWorker
  Virtual base class providing different methods to be adapted.
*/
class VISP_EXPORT vpWriterWorker
{
public:
  vpWriterWorker() = default;

  virtual void init() = 0;

  virtual void run() = 0;

  virtual bool runOnce() = 0;

private:
};

#endif
