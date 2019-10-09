/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
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
 * Matrix generalized multiplication.
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/
#include <visp3/core/vpGEMM.h>

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
#  ifdef VISP_HAVE_MKL
#    include <mkl.h>
typedef MKL_INT integer;
#  else
#    include <cblas.h>
typedef int integer;
#  endif
#endif

namespace {

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
void blas_dgemm(char trans_a_, char trans_b_, int M_, int N_, int K_, double alpha,
                const double *a_data, int lda_, const double *b_data, int ldb_, double beta,
                double *c_data, int ldc_)
{
  const integer M = (integer)M_, K = (integer)K_, N = (integer)N_;
  const integer lda = (integer)lda_, ldb = (integer)ldb_, ldc = (integer)ldc_;
  CBLAS_TRANSPOSE trans_a = CblasNoTrans;
  if (trans_a_ == 'T' || trans_a_ == 't') {
    trans_a = CblasTrans;
  } else if (trans_a_ == 'C' || trans_a_ == 'c') {
    trans_a = CblasConjTrans;
  }

  CBLAS_TRANSPOSE trans_b = CblasNoTrans;
  if (trans_b_ == 'T' || trans_b_ == 't') {
    trans_b = CblasTrans;
  } else if (trans_b_ == 'C' || trans_b_ == 'c') {
    trans_b = CblasConjTrans;
  }

  cblas_dgemm(CblasRowMajor, trans_a, trans_b, M, N, K, alpha, a_data, lda, b_data, ldb, beta, c_data, ldc);
}

void blas_dgemv(char trans_, int M_, int N_, double alpha, const double *a_data, int lda_,
                const double *x_data, int incx_, double beta, double *y_data, int incy_)
{
  const integer M = (integer)M_, N = (integer)N_;
  const integer lda = (integer)lda_, incx = (integer)incx_, incy = (integer)incy_;
  CBLAS_TRANSPOSE trans = CblasNoTrans;
  if (trans_ == 'T' || trans_ == 't') {
    trans = CblasTrans;
  } else if (trans_ == 'C' || trans_ == 'c') {
    trans = CblasConjTrans;
  }

  cblas_dgemv(CblasRowMajor, trans, M, N, alpha, a_data, lda, x_data, incx, beta, y_data, incy);
}
#endif

} //namespace

class vpBLAS::Impl {
public:
  Impl() { }

  void dgemm(const vpArray2D<double>& A, const vpArray2D<double>& B, double alpha,
             vpArray2D<double>& C, double beta, int ops)
  {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
  switch (ops) {
  case 0:
    blas_dgemm('N', 'N', A.getRows(), B.getCols(), A.getCols(), alpha, A.data, A.getCols(), B.data, B.getCols(), beta, C.data, C.getCols());
    break;
  case 1:
    blas_dgemm('T', 'N', A.getCols(), B.getCols(), A.getRows(), alpha, A.data, A.getCols(), B.data, B.getCols(), beta, C.data, C.getCols());
    break;
  case 2:
    blas_dgemm('N', 'T', A.getRows(), B.getRows(), A.getCols(), alpha, A.data, A.getCols(), B.data, B.getCols(), beta, C.data, C.getCols());
    break;
  case 3:
    blas_dgemm('T', 'T', A.getCols(), B.getRows(), A.getRows(), alpha, A.data, A.getCols(), B.data, B.getCols(), beta, C.data, C.getCols());
    break;
  case 4:
  {
    vpArray2D<double> C_const = C;
    vpTGEMM<4>(A, B, alpha, C_const, beta, C);
  }
    break;
  case 5:
  {
    vpArray2D<double> C_const = C;
    vpTGEMM<5>(A, B, alpha, C_const, beta, C);
  }
    break;
  case 6:
  {
    vpArray2D<double> C_const = C;
    vpTGEMM<6>(A, B, alpha, C_const, beta, C);
  }
    break;
  case 7:
  {
    vpArray2D<double> C_const = C;
    vpTGEMM<7>(A, B, alpha, C_const, beta, C);
  }
    break;
  default:
    throw(vpException(vpException::functionNotImplementedError, "Operation on vpGEMM not implemented"));
  }
#else
  vpArray2D<double> C_const = C;
  switch (ops) {
  case 0:
    vpTGEMM<0>(A, B, alpha, C_const, beta, C);
    break;
  case 1:
    vpTGEMM<1>(A, B, alpha, C_const, beta, C);
    break;
  case 2:
    vpTGEMM<2>(A, B, alpha, C_const, beta, C);
    break;
  case 3:
    vpTGEMM<3>(A, B, alpha, C_const, beta, C);
    break;
  case 4:
    vpTGEMM<4>(A, B, alpha, C_const, beta, C);
    break;
  case 5:
    vpTGEMM<5>(A, B, alpha, C_const, beta, C);
    break;
  case 6:
    vpTGEMM<6>(A, B, alpha, C_const, beta, C);
    break;
  case 7:
    vpTGEMM<7>(A, B, alpha, C_const, beta, C);
    break;
  default:
    throw(vpException(vpException::functionNotImplementedError, "Operation on vpGEMM not implemented"));
  }
#endif
  }

  void dgemv(const vpMatrix& A, double alpha, const vpColVector& x, double beta, vpColVector& y, bool transA)
  {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
  const char trans = transA ? 'T' : 'N';
  const int incx = 1, incy = 1;
  blas_dgemv(trans, A.getRows(), A.getCols(), alpha, A.data, A.getCols(), x.data, incx, beta, y.data, incy);
#else
  y = (transA ? A.t() : A) * x;
#endif
  }
};

vpBLAS::vpBLAS() : impl_(new Impl())
{
}

vpBLAS::~vpBLAS()
{
  delete impl_;
}

/*!
   This function performs generalized matrix multiplication:
   C = alpha*op(A)*op(B) + beta*C, where op(X) is X or X^T.
   Operation on A, B and C matrices is described by enumeration
   vpGEMMmethod().

   For example, to compute C = alpha*A^T*B^T+beta*C we need to call :
   \code
   vpBLAS blas;
   blas.dgemm(A, B, alpha, C, beta, VP_GEMM_A_T + VP_GEMM_B_T);
   \endcode

   \exception vpException::incorrectMatrixSizeError if the sizes of the
   matrices do not allow the operations.

   \param A : An array that could be a vpMatrix.
   \param B : An array that could be a vpMatrix.
   \param alpha : A scalar.
   \param C : An array that could be a vpMatrix.
   \param beta : A scalar.
   \param ops : A scalar describing operation applied on the matrices.
   Possible values are the one defined in vpGEMMmethod(): VP_GEMM_A_T,
   VP_GEMM_B_T.

   \relates vpArray2D
*/
void vpBLAS::dgemm(const vpArray2D<double>& A, const vpArray2D<double>& B, double alpha,
                    vpArray2D<double>& C, double beta, int ops)
{
  impl_->dgemm(A, B, alpha, C, beta, ops);
}

/*!
   This function performs generalized matrix vector multiplication:
   y = alpha*A*x + beta if transA is false,
   y = alpha*A^T + beta if transA is true.

   \param A : An (MxN) vpMatrix.
   \param alpha : Alpha value.
   \param x : An (Nx1) vpColVector.
   \param beta : Beta value.
   \param y : Result of the matrix vector multiplication.
   \param transA : If true, A will be transposed.
*/
void vpBLAS::dgemv(const vpMatrix& A, double alpha, const vpColVector& x, double beta, vpColVector& y, bool transA)
{
  impl_->dgemv(A, alpha, x, beta, y, transA);
}
