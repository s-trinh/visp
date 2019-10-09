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
 * Benchmark matrix multiplication.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpGEMM.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

namespace {

double getRandomValues(double min, double max)
{
  return (max - min) * ((double)rand() / (double)RAND_MAX) + min;
}

vpMatrix generateRandomMatrix(unsigned int rows, unsigned int cols, double min=-1, double max=1)
{
  vpMatrix M(rows, cols);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = getRandomValues(min, max);
    }
  }

  return M;
}

vpColVector generateRandomVector(unsigned int rows, double min=-1, double max=1)
{
  vpColVector v(rows);

  for (unsigned int i = 0; i < v.getRows(); i++) {
    v[i] = getRandomValues(min, max);
  }

  return v;
}

bool equalMatrix(const vpMatrix& A, const vpMatrix& B, double tol=1e-9)
{
  if (A.getRows() != B.getRows() || A.getCols() != B.getCols()) {
    std::cerr << "Size error" << std::endl;
    return false;
  }

  for (unsigned int i = 0; i < A.getRows(); i++) {
    for (unsigned int j = 0; j < A.getCols(); j++) {
      if (!vpMath::equal(A[i][j], B[i][j], tol)) {
        return false;
      }
    }
  }

  return true;
}

bool equalVector(const vpColVector& A, const vpColVector& B, double tol=1e-9)
{
  if (A.getRows() != B.getRows() || A.getCols() != B.getCols()) {
    std::cerr << "Size error" << std::endl;
    return false;
  }

  for (unsigned int i = 0; i < A.getRows(); i++) {
    if (!vpMath::equal(A[i], B[i], tol)) {
      return false;
    }
  }

  return true;
}

vpMatrix matMul(const vpMatrix& A, const vpMatrix& B)
{
  vpMatrix C(A.getRows(), B.getCols());
  unsigned int BcolNum = B.getCols();
  unsigned int BrowNum = B.getRows();

  for (unsigned int i = 0; i < A.getRows(); i++) {
    double *rowptri = A[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < BcolNum; j++) {
      double s = 0;
      for (unsigned int k = 0; k < BrowNum; k++) {
        s += rowptri[k] * B[k][j];
      }
      ci[j] = s;
    }
  }

  return C;
}

vpColVector matVecMul(const vpMatrix& A, const vpColVector& x)
{
  vpColVector y(A.getRows(), 0.0);

  for (unsigned int j = 0; j < A.getCols(); j++) {
    double vj = x[j];
    for (unsigned int i = 0; i < A.getRows(); i++) {
      y[i] += A[i][j] * vj;
    }
  }

  return y;
}

}

TEST_CASE("vpGEMM: AxB", "[vpGEMM]") {
  const int M = 31, K = 47, N = 25;

  vpMatrix A = generateRandomMatrix(M, K);
  vpMatrix B = generateRandomMatrix(K, N);
  vpMatrix C = generateRandomMatrix(M, N);

  vpMatrix res_AB_truth = matMul(A, B);
  vpMatrix res_AB_gemm = C;
  vpBLAS blas;
  blas.dgemm(A, B, 1, res_AB_gemm, 0);

  REQUIRE(equalMatrix(res_AB_truth, res_AB_gemm));
}

TEST_CASE("vpGEMM: A^TxB", "[vpGEMM]") {
  const int M = 31, K = 47, N = 25;

  vpMatrix A = generateRandomMatrix(K, M);
  vpMatrix B = generateRandomMatrix(K, N);
  vpMatrix C = generateRandomMatrix(M, N);

  vpMatrix res_AtB_truth = matMul(A.t(), B);
  vpMatrix res_AtB_gemm(C.getRows(), C.getCols());
  vpBLAS blas;
  blas.dgemm(A, B, 1, res_AtB_gemm, 0, VP_GEMM_A_T);

  REQUIRE(equalMatrix(res_AtB_truth, res_AtB_gemm));
}

TEST_CASE("vpGEMM: AxB^T", "[vpGEMM]") {
  const int M = 31, K = 47, N = 25;

  vpMatrix A = generateRandomMatrix(M, K);
  vpMatrix B = generateRandomMatrix(N, K);
  vpMatrix C = generateRandomMatrix(M, N);

  vpMatrix res_ABt_truth = matMul(A, B.t());
  vpMatrix res_ABt_gemm(C.getRows(), C.getCols());
  vpBLAS blas;
  blas.dgemm(A, B, 1, res_ABt_gemm, 0, VP_GEMM_B_T);

  REQUIRE(equalMatrix(res_ABt_truth, res_ABt_gemm));
}

TEST_CASE("vpGEMM: A^TxB^T", "[vpGEMM]") {
  const int M = 31, K = 47, N = 25;

  vpMatrix A = generateRandomMatrix(K, M);
  vpMatrix B = generateRandomMatrix(N, K);
  vpMatrix C = generateRandomMatrix(M, N);

  vpMatrix res_AtBt_truth = matMul(A.t(), B.t());
  vpMatrix res_AtBt_gemm(C.getRows(), C.getCols());
  vpBLAS blas;
  blas.dgemm(A, B, 1, res_AtBt_gemm, 0, VP_GEMM_A_T + VP_GEMM_B_T);

  REQUIRE(equalMatrix(res_AtBt_truth, res_AtBt_gemm));
}

TEST_CASE("vpGEMM: Ax", "[vpGEMM]") {
  const int M = 31, N = 47;
  const double alpha = 1.0, beta = 0.0;

  vpMatrix A = generateRandomMatrix(M, N);
  vpColVector x = generateRandomVector(N);

  vpColVector y_true = matVecMul(A, x);
  vpColVector y_gemv;
  y_gemv.resize(M, false);
  vpBLAS blas;
  blas.dgemv(A, alpha, x, beta, y_gemv);

  REQUIRE(equalVector(y_true, y_gemv));
}

TEST_CASE("vpGEMM: A^Tx", "[vpGEMM]") {
  const int M = 31, N = 47;
  const double alpha = 1.0, beta = 0.0;
  const bool transA = true;

  vpMatrix A = generateRandomMatrix(N, M);
  vpColVector x = generateRandomVector(N);

  vpColVector y_true = matVecMul(A.t(), x);
  vpColVector y_gemv;
  y_gemv.resize(M, false);
  vpBLAS blas;
  blas.dgemv(A, alpha, x, beta, y_gemv, transA);

  REQUIRE(equalVector(y_true, y_gemv));
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  using namespace Catch::clara;
  bool opt_click = false, opt_display = false;
  auto cli
      = session.cli()
      | Opt(opt_click)
      ["-c"]
      ("ctest -c option?")
      | Opt(opt_display)
      ["-d"]
      ("ctest -d option?");

  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  int returnCode = session.applyCommandLine( argc, argv );

  if(returnCode != 0) { // Indicates a command line error
    return returnCode;
  }

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() {
  return 0;
}
#endif
