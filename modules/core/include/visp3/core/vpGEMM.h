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

#ifndef _vpGEMM_h_
#define _vpGEMM_h_

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpException.h>

/*!
  Enumeration of the operations applied on matrices in vpGEMM() function.

  Operations are :
  - VP_GEMM_A_T to use the transpose matrix of A instead of the matrix A
  - VP_GEMM_B_T to use the transpose matrix of B instead of the matrix B
  - VP_GEMM_C_T to use the transpose matrix of C instead of the matrix C

  \relates vpArray2D
  */
enum {
  VP_GEMM_A_T = 1, //! Use A^T instead of A
  VP_GEMM_B_T = 2, //! Use B^T instead of B
  VP_GEMM_C_T = 4, //! Use C^T instead of C
} vpGEMMmethod;

namespace {

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
#  ifdef VISP_HAVE_MKL
#include <mkl.h>

void blas_dgemm(char trans_a_, char trans_b_, int M_, int N_, int K_, double alpha,
                const double *a_data, int lda_, const double *b_data, int ldb_, double beta,
                double *c_data, const int ldc_)
{
  const MKL_INT M = (MKL_INT)M_, K = (MKL_INT)K_, N = (MKL_INT)N_;
  const MKL_INT lda = (MKL_INT)lda_, ldb = (MKL_INT)ldb_, ldc = (MKL_INT)ldc_;
  CBLAS_TRANSPOSE trans_a = CblasNoTrans;
  if (trans_a_ == 'T' || trans_a_ == 't') {
    trans_a = CblasTrans;
  } else if (trans_a_ == 'C' || trans_a_ == 'c') {
    trans_a = CblasConjTrans;
  }
  CBLAS_TRANSPOSE trans_b = CblasNoTrans;
  if (trans_b_ == 'T' || trans_b_ == 't') {
    trans_b = CblasTrans;
  }
  else if (trans_b_ == 'C' || trans_b_ == 'c') {
    trans_b = CblasConjTrans;
  }

  cblas_dgemm(CblasRowMajor, trans_a, trans_b, M, N, K, alpha, a_data, lda, b_data, ldb, beta, c_data, ldc);
}

void blas_dgemv(char trans_, int M_, int N_, double alpha, const double *a_data, int lda_,
                const double *x_data, int incx_, double beta, double *y_data, int incy_)
{
  const MKL_INT M = (MKL_INT)M_, N = (MKL_INT)N_;
  const MKL_INT lda = (MKL_INT)lda_, incx = (MKL_INT)incx_, incy = (MKL_INT)incy_;
  CBLAS_TRANSPOSE trans = CblasNoTrans;
  if (trans_ == 'T' || trans_ == 't') {
    trans = CblasTrans;
  } else if (trans_ == 'C' || trans_ == 'c') {
    trans = CblasConjTrans;
  }

  cblas_dgemv(CblasRowMajor, trans, M, N, alpha, a_data, lda, x_data, incx, beta, y_data, incy);
}
#  else
typedef int integer;

extern "C" void dgemm_(char *transa, char *transb, integer *M, integer *N, integer *K, double *alpha, double *a,
  integer *lda, double *b, integer *ldb, double *beta, double *c, integer *ldc);

extern "C" void dgemv_(char *trans, integer *M, integer *N, double *alpha, double *a, integer *lda, double *x,
  integer *incx, double *beta, double *y, integer *incy);

void blas_dgemm(char trans_a, char trans_b, const int M_, const int N_, const int K_, double alpha,
                double *a_data, const int lda_, double *b_data, const int ldb_, double beta, double *c_data,
                const int ldc_)
{
  integer M = (integer)M_, K = (integer)K_, N = (integer)N_;
  integer lda = (integer)lda_, ldb = (integer)ldb_, ldc = (integer)ldc_;

  dgemm_(&trans_a, &trans_b, &M, &N, &K, &alpha, a_data, &lda, b_data, &ldb, &beta, c_data, &ldc);
}

void blas_dgemv(char trans, const int M_, const int N_, double alpha, double *a_data, const int lda_,
                double *x_data, const int incx_, double beta, double *y_data, const int incy_)
{
  integer M = (integer)M_, N = (integer)N_;
  integer lda = (integer)lda_, incx = (integer)incx_, incy = (integer)incy_;

  dgemv_(&trans, &M, &N, &alpha, a_data, &lda, x_data, &incx, &beta, y_data, &incy);
}
#  endif
#endif

template <unsigned int>
inline void GEMMsize(const vpArray2D<double> & /*A*/, const vpArray2D<double> & /*B*/, unsigned int & /*Arows*/,
                     unsigned int & /*Acols*/, unsigned int & /*Brows*/, unsigned int & /*Bcols*/)
{
}

template <>
void inline GEMMsize<0>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getRows();
  Acols = A.getCols();
  Brows = B.getRows();
  Bcols = B.getCols();
}

template <>
inline void GEMMsize<1>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getCols();
  Acols = A.getRows();
  Brows = B.getRows();
  Bcols = B.getCols();
}
template <>
inline void GEMMsize<2>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getRows();
  Acols = A.getCols();
  Brows = B.getCols();
  Bcols = B.getRows();
}
template <>
inline void GEMMsize<3>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getCols();
  Acols = A.getRows();
  Brows = B.getCols();
  Bcols = B.getRows();
}

template <>
inline void GEMMsize<4>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getRows();
  Acols = A.getCols();
  Brows = B.getRows();
  Bcols = B.getCols();
}

template <>
inline void GEMMsize<5>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getCols();
  Acols = A.getRows();
  Brows = B.getRows();
  Bcols = B.getCols();
}

template <>
inline void GEMMsize<6>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getRows();
  Acols = A.getCols();
  Brows = B.getCols();
  Bcols = B.getRows();
}

template <>
inline void GEMMsize<7>(const vpArray2D<double> &A, const vpArray2D<double> &B, unsigned int &Arows,
                        unsigned int &Acols, unsigned int &Brows, unsigned int &Bcols)
{
  Arows = A.getCols();
  Acols = A.getRows();
  Brows = B.getCols();
  Bcols = B.getRows();
}

template <unsigned int>
inline void GEMM1(const unsigned int & /*Arows*/, const unsigned int & /*Brows*/, const unsigned int & /*Bcols*/,
                  const vpArray2D<double> & /*A*/, const vpArray2D<double> & /*B*/, const double & /*alpha*/,
                  vpArray2D<double> & /*D*/)
{
}

template <>
inline void GEMM1<0>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[n][c] * alpha;
      D[r][c] = sum;
    }
}

template <>
inline void GEMM1<1>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[n][c] * alpha;
      D[r][c] = sum;
    }
}

template <>
inline void GEMM1<2>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[c][n] * alpha;
      D[r][c] = sum;
    }
}

template <>
inline void GEMM1<3>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[c][n] * alpha;
      D[r][c] = sum;
    }
}

template <unsigned int>
inline void GEMM2(const unsigned int & /*Arows*/, const unsigned int & /*Brows*/, const unsigned int & /*Bcols*/,
                  const vpArray2D<double> & /*A*/, const vpArray2D<double> & /*B*/, const double & /*alpha*/,
                  const vpArray2D<double> & /*C*/, const double & /*beta*/, vpArray2D<double> & /*D*/)
{
}

template <>
inline void GEMM2<0>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[n][c] * alpha;
      D[r][c] = sum + C[r][c] * beta;
    }
}

template <>
inline void GEMM2<1>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[n][c] * alpha;
      D[r][c] = sum + C[r][c] * beta;
    }
}

template <>
inline void GEMM2<2>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[c][n] * alpha;
      D[r][c] = sum + C[r][c] * beta;
    }
}

template <>
inline void GEMM2<3>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[c][n] * alpha;
      D[r][c] = sum + C[r][c] * beta;
    }
}

template <>
inline void GEMM2<4>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[n][c] * alpha;
      D[r][c] = sum + C[c][r] * beta;
    }
}

template <>
inline void GEMM2<5>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[n][c] * alpha;
      D[r][c] = sum + C[c][r] * beta;
    }
}

template <>
inline void GEMM2<6>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[r][n] * B[c][n] * alpha;
      D[r][c] = sum + C[c][r] * beta;
    }
}

template <>
inline void GEMM2<7>(const unsigned int &Arows, const unsigned int &Brows, const unsigned int &Bcols,
                     const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                     const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  for (unsigned int r = 0; r < Arows; r++)
    for (unsigned int c = 0; c < Bcols; c++) {
      double sum = 0;
      for (unsigned int n = 0; n < Brows; n++)
        sum += A[n][r] * B[c][n] * alpha;
      D[r][c] = sum + C[c][r] * beta;
    }
}

template <unsigned int T>
inline void vpTGEMM(const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                    const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D)
{
  unsigned int Arows;
  unsigned int Acols;
  unsigned int Brows;
  unsigned int Bcols;

  GEMMsize<T>(A, B, Arows, Acols, Brows, Bcols);

  try {
    if ((Arows != D.getRows()) || (Bcols != D.getCols()))
      D.resize(Arows, Bcols);
  } catch (...) {
    throw;
  }

  if (Acols != Brows) {
    throw(vpException(vpException::dimensionError, "In vpGEMM, cannot multiply (%dx%d) matrix by (%dx%d) matrix", Arows,
                      Acols, Brows, Bcols));
  }

  if (C.getRows() != 0 && C.getCols() != 0) {
    if ((Arows != C.getRows()) || (Bcols != C.getCols())) {
      throw(vpException(vpException::dimensionError, "In vpGEMM, cannot add resulting (%dx%d) matrix to (%dx%d) matrix",
                        Arows, Bcols, C.getRows(), C.getCols()));
    }

    GEMM2<T>(Arows, Brows, Bcols, A, B, alpha, C, beta, D);
  } else {
    GEMM1<T>(Arows, Brows, Bcols, A, B, alpha, D);
  }
}

} //namespace

/*!
   This function performs generalized matrix multiplication:
   D = alpha*op(A)*op(B) + beta*op(C), where op(X) is X or X^T.
   Operation on A, B and C matrices is described by enumeration
   vpGEMMmethod().

   For example, to compute D = alpha*A^T*B^T+beta*C we need to call :
   \code
   vpGEMM(A, B, alpha, C, beta, D, VP_GEMM_A_T + VP_GEMM_B_T);
   \endcode

   If C is not used, vpGEMM must be called using an empty array \e null.
   Thus to compute D = alpha*A^T*B, we have to call:
   \code
   vpGEMM(A, B, alpha, null, 0, D, VP_GEMM_B_T);
   \endcode

   \exception vpException::incorrectMatrixSizeError if the sizes of the
   matrices do not allow the operations.

   \param A : An array that could be a vpMatrix.
   \param B : An array that could be a vpMatrix.
   \param alpha : A scalar.
   \param C : An array that could be a vpMatrix.
   \param beta : A scalar.
   \param D : The resulting array that could be a vpMatrix.
   \param ops : A scalar describing operation applied on the matrices.
   Possible values are the one defined in vpGEMMmethod(): VP_GEMM_A_T,
   VP_GEMM_B_T, VP_GEMM_C_T.

   \relates vpArray2D
*/
inline void vpGEMM(const vpArray2D<double> &A, const vpArray2D<double> &B, const double &alpha,
                   const vpArray2D<double> &C, const double &beta, vpArray2D<double> &D, const unsigned int &ops = 0)
{
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
  switch (ops) {
  case 0:
    blas_dgemm('N', 'N', A.getRows(), B.getCols(), A.getCols(), alpha, A.data, A.getRows(), B.data, B.getRows(), beta, C.data, C.getRows());
    break;
  case 1:
    vpTGEMM<1>(A, B, alpha, C, beta, D);
    break;
  case 2:
    vpTGEMM<2>(A, B, alpha, C, beta, D);
    break;
  case 3:
    vpTGEMM<3>(A, B, alpha, C, beta, D);
    break;
  case 4:
    vpTGEMM<4>(A, B, alpha, C, beta, D);
    break;
  case 5:
    vpTGEMM<5>(A, B, alpha, C, beta, D);
    break;
  case 6:
    vpTGEMM<6>(A, B, alpha, C, beta, D);
    break;
  case 7:
    vpTGEMM<7>(A, B, alpha, C, beta, D);
    break;
  default:
    throw(vpException(vpException::functionNotImplementedError, "Operation on vpGEMM not implemented"));
    break;
  }
#else
  switch (ops) {
  case 0:
    vpTGEMM<0>(A, B, alpha, C, beta, D);
    break;
  case 1:
    vpTGEMM<1>(A, B, alpha, C, beta, D);
    break;
  case 2:
    vpTGEMM<2>(A, B, alpha, C, beta, D);
    break;
  case 3:
    vpTGEMM<3>(A, B, alpha, C, beta, D);
    break;
  case 4:
    vpTGEMM<4>(A, B, alpha, C, beta, D);
    break;
  case 5:
    vpTGEMM<5>(A, B, alpha, C, beta, D);
    break;
  case 6:
    vpTGEMM<6>(A, B, alpha, C, beta, D);
    break;
  case 7:
    vpTGEMM<7>(A, B, alpha, C, beta, D);
    break;
  default:
    throw(vpException(vpException::functionNotImplementedError, "Operation on vpGEMM not implemented"));
    break;
  }
#endif
}

#endif
