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
 * Test threading capabilities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \example testThread.cpp

  \brief Test threading capabilities.

*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PTHREAD) || defined(_WIN32)

//! [Code]
#include <iostream>

#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/io/vpImageIo.h>

vpThread::Return myFooFunction(vpThread::Args args)
{
  (void)(args); // Avoid warning: unused parameter args
  // do stuff...
  return 0;
}

vpThread::Return myBarFunction(vpThread::Args args)
{
  (void)(args); // Avoid warning: unused parameter args
  // do stuff...
  return 0;
}

vpThread::Return myQuxFunction(vpThread::Args args)
{
  unsigned int args_ = *((unsigned int *) args);
  std::cout << "qux arg: " << args_ << std::endl;
  // do stuff...
  return 0;
}

struct thread_data
{
  int m_id;
  thread_data(int id) : m_id(id) {}
};

DWORD WINAPI thread_func(LPVOID lpParameter)
{
  thread_data *td = (thread_data*) lpParameter;
  std::cout << "thread with id = " << td->m_id << std::endl;

  unsigned int myCounter = 0;
  while(myCounter < 0xFFFFFFFF) ++myCounter;
  std::cout << "myCounter=" << myCounter << std::endl;

  return 0;
}

struct Param_Thread_t {
  const vpImage<unsigned char> * const m_I;
  vpImage<unsigned char> * m_I_res;

  Param_Thread_t() : m_I(NULL), m_I_res(NULL) {
  }

  Param_Thread_t(const vpImage<unsigned char> * const I) : m_I(I), m_I_res(NULL) {
  }
};

vpThread::Return threadGaussianBlur(vpThread::Args args) {
  Param_Thread_t *param = ( (Param_Thread_t *) args );

  vpImage<double> I_gaussian;
  vpImageFilter::gaussianBlur(*param->m_I, I_gaussian, 21);

  double minValue, maxValue;
  I_gaussian.getMinMaxValue(minValue, maxValue);

  param->m_I_res = new vpImage<unsigned char>(I_gaussian.getHeight(), I_gaussian.getWidth());
  vpImage<unsigned char> *I_gaussian_save = param->m_I_res;
  for(unsigned int i = 0; i < I_gaussian_save->getHeight(); i++) {
    for(unsigned int j = 0; j < I_gaussian_save->getWidth(); j++) {
      (*I_gaussian_save)[i][j] = vpMath::saturate<unsigned char>(255.0 / (maxValue-minValue) * I_gaussian[i][j] - -255.0*minValue / (maxValue-minValue));
    }
  }

  return 0;
}

int main() 
{
  unsigned int qux_arg = 12;
  vpThread foo;
  vpThread bar((vpThread::Fn)myBarFunction);
  vpThread qux((vpThread::Fn)myQuxFunction, (vpThread::Args)&qux_arg); // Pass qux_arg to myQuxFunction() function

  vpTime::wait(1000); // Sleep 1s to ensure myQuxFunction() internal printings
  std::cout << "Joinable after construction:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;

  foo.create((vpThread::Fn)myFooFunction);

  std::cout << "Joinable after creation:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;

  if (foo.joinable()) foo.join();
  if (bar.joinable()) bar.join();
  if (qux.joinable()) qux.join();

  std::cout << "Joinable after joining:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;



  //for (int i=0; i< 10; i++)
  //{
  //  CreateThread(NULL, 0, thread_func, new thread_data(i) , 0, 0);
  //}


  vpImage<unsigned char> I_load;
  vpImageIo::read(I_load, "C:/Users/strinh/Pictures/16334511258_0fd36895b0_o.jpg");
  vpImage<unsigned char> *I = &I_load;

  std::vector<vpThread *> threadpool;
  std::vector<Param_Thread_t *> threadParams;

  Param_Thread_t *thread_param = NULL;
  vpThread *thread = NULL;
  
  int nbThreads = 4;
  double t = vpTime::measureTimeMs();
  for(int  i = 0; i < nbThreads; i++) {
    thread_param = new Param_Thread_t(I);
    threadParams.push_back(thread_param);

    // Start the threads
    thread = new vpThread((vpThread::Fn) threadGaussianBlur, (vpThread::Args) thread_param);
    threadpool.push_back(thread);
  }

  for(size_t cpt = 0; cpt < threadpool.size(); cpt++) {
    // Wait until thread ends up
    threadpool[cpt]->join();
  }

  t = vpTime::measureTimeMs() - t;
  std::cout << "\nt=" << t << " ms" << std::endl;

  std::vector<vpImage<unsigned char> *> vectorOfRes;
  double t_regular = vpTime::measureTimeMs();
  for(int i = 0; i < nbThreads; i++) {
    vpImage<double> I_gaussian;
    vpImageFilter::gaussianBlur(I_load, I_gaussian, 21);

    double minValue, maxValue;
    I_gaussian.getMinMaxValue(minValue, maxValue);

    vpImage<unsigned char> *I_gaussian_save = new vpImage<unsigned char>(I_gaussian.getHeight(), I_gaussian.getWidth());
    for(unsigned int i = 0; i < I_gaussian_save->getHeight(); i++) {
      for(unsigned int j = 0; j < I_gaussian_save->getWidth(); j++) {
        (*I_gaussian_save)[i][j] = vpMath::saturate<unsigned char>(255.0 / (maxValue-minValue) * I_gaussian[i][j] - -255.0*minValue / (maxValue-minValue));
      }
    }

    vectorOfRes.push_back(I_gaussian_save);
  }
  t_regular = vpTime::measureTimeMs() - t_regular;
  std::cout << "t_regular=" << t_regular << " ms" << std::endl;
  std::cout << "Speedup=" << t_regular/t << "X" << std::endl;

  for(size_t cpt = 0; cpt < vectorOfRes.size(); cpt++) {
    std::stringstream ss;
    ss << "I_gaussian_res_regular_" << cpt << ".png";

    vpImageIo::write(*vectorOfRes[cpt], ss.str());
  }

  //Save
  for(size_t cpt = 0; cpt < threadParams.size(); cpt++) {
    std::stringstream ss;
    ss << "I_gaussian_res_" << cpt << ".png";

    vpImageIo::write(*threadParams[cpt]->m_I_res, ss.str());
  }

  //Delete
  for(size_t cpt = 0; cpt < threadpool.size(); cpt++) {
    delete threadpool[cpt];
  }

  for(size_t cpt = 0; cpt < threadParams.size(); cpt++) {
    delete threadParams[cpt];
  }

  return 0;
}
//! [Code]

#else

#include <iostream>

int main()
{
#  if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#  else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
}
#endif
