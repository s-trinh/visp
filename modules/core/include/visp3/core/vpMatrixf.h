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
 * Matrix manipulation.
 *
 *****************************************************************************/

#ifndef vpMatrixf_H
#define vpMatrixf_H

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpForceTwistMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#include <iostream>
#include <math.h>

class vpRowVector;
class vpColVectorf;
class vpTranslationVector;
class vpHomogeneousMatrix;
class vpVelocityTwistMatrix;
class vpForceTwistMatrix;

/*!
  \file vpMatrixf.h

  \brief Definition of matrix class as well as a set of operations on
  these matrices.
*/

/*!
  \class vpMatrixf
  \ingroup group_core_matrices

  \brief Implementation of a matrix and operations on matrices.

  This class needs one of the following third-party to compute matrix inverse,
  pseudo-inverse, singular value decomposition, determinant:
  - If Lapack is installed and detected by ViSP, this 3rd party is used by
  vpMatrixf. Installation instructions are provided here
  https://visp.inria.fr/3rd_lapack;
  - else if Eigen3 is installed and detected by ViSP, this 3rd party is used
  by vpMatrixf. Installation instructions are provided here
  https://visp.inria.fr/3rd_eigen;
  - else if OpenCV is installed and detected by ViSP, this 3rd party is used,
    Installation instructions are provided here
  https://visp.inria.fr/3rd_opencv;
  - If none of these previous 3rd parties is installed, we use by default a
  Lapack built-in version.

  vpMatrixf class provides a data structure for the matrices as well
  as a set of operations on these matrices.

  The vpMatrixf class is derived from vpArray2D<float>.

  The code below shows how to create a 2-by-3 matrix of floats, set the element values and access them:
  \code
#include <visp3/code/vpMatrixf.h

int main()
{
  vpMatrixf M(2, 3);
  M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
  M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

  std::cout << "M:" << std::endl;
  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      std::cout << M[i][j] << " ";
    }
    std::cout << std::endl;
  }
}
  \endcode
  Once build, this previous code produces the following output:
  \code
M:
-1 -2 -3
4 5.5 6
  \endcode
  If ViSP is build with c++11 enabled, you can do the same using:
  \code
#include <visp3/code/vpMatrixf.h

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrixf M( {-1, -2, -3}, {4, 5.5, 6.0f} );
  std::cout << "M:\n" << M << std::endl;
#endif
}
  \endcode
  You can also create and initialize a matrix this way:
  \code
#include <visp3/code/vpMatrixf.h

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrixf M(2, 3, {-1, -2, -3, 4, 5.5, 6.0f} );
#endif
}
  \endcode

  The Matrix could also be initialized using operator=(const std::initializer_list< std::initializer_list< float > > &)
  \code
int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrixf M;
  M = { {-1, -2, -3}, {4, 5.5, 6.0f} };
#endif
}
  \endcode

  \sa vpArray2D, vpRowVector, vpColVectorf, vpHomogeneousMatrix,
  vpRotationMatrix, vpVelocityTwistMatrix, vpForceTwistMatrix, vpHomography
*/
class VISP_EXPORT vpMatrixf : public vpArray2D<float>
{
public:
  /*!
    Method used to compute the determinant of a square matrix.
    \sa det()
  */
  typedef enum {
    LU_DECOMPOSITION /*!< LU decomposition method. */
  } vpDetMethod;

public:
  /*!
    Basic constructor of a matrix of float. Number of columns and rows are
    zero.
  */
  vpMatrixf() : vpArray2D<float>(0, 0) {}

  /*!
    Constructor that initialize a matrix of float with 0.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
  */
  vpMatrixf(unsigned int r, unsigned int c) : vpArray2D<float>(r, c) {}

  /*!
    Constructor that initialize a matrix of float with \e val.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
    \param val : Each element of the matrix is set to \e val.
  */
  vpMatrixf(unsigned int r, unsigned int c, float val) : vpArray2D<float>(r, c, val) {}
  vpMatrixf(const vpMatrixf &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols);

  /*!
     Create a matrix from a 2D array that could be one of the following
     container that inherit from vpArray2D such as vpMatrixf, vpRotationMatrix,
     vpHomogeneousMatrix, vpPoseVector, vpColVectorf, vpRowVector...

     The following example shows how to create a matrix from an homogeneous
     matrix:
\code
vpRotationMatrix R;
vpMatrixf M(R);
\endcode
   */
  vpMatrixf(const vpArray2D<float> &A) : vpArray2D<float>(A) {}

  vpMatrixf(const vpMatrixf &A) : vpArray2D<float>(A) {}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrixf(vpMatrixf &&A);
  explicit vpMatrixf(const std::initializer_list<float> &list);
  explicit vpMatrixf(unsigned int nrows, unsigned int ncols, const std::initializer_list<float> &list);
  explicit vpMatrixf(const std::initializer_list<std::initializer_list<float> > &lists);
#endif

  //! Destructor (Memory de-allocation)
  virtual ~vpMatrixf() {}

  /*!
    Removes all elements from the matrix (which are destroyed),
    leaving the container with a size of 0.
  */
  void clear()
  {
    if (data != NULL) {
      free(data);
      data = NULL;
    }

    if (rowPtrs != NULL) {
      free(rowPtrs);
      rowPtrs = NULL;
    }
    rowNum = colNum = dsize = 0;
  }

  //-------------------------------------------------
  // Setting a diagonal matrix
  //-------------------------------------------------
  /** @name Linear algebra optimization  */
  //@{
  /*!
   * Return the minimum size of rows and columns required to enable Blas/Lapack
   * usage on matrices and vectors.
   *
   * To get more info see \ref tutorial-basic-linear-algebra.
   *
   * \sa setLapackMatrixMinSize()
   */
  static unsigned int getLapackMatrixMinSize() { return m_lapack_min_size; }

  /*!
   * Modify default size used to determine if Blas/Lapack basic linear algebra operations are enabled.
   *
   * To get more info see \ref tutorial-basic-linear-algebra.
   *
   * \param min_size : Minimum size of rows and columns required for a matrix or a vector to use
   * Blas/Lapack third parties like MKL, OpenBLAS, Netlib or Atlas. When matrix or vector size is
   * lower or equal to this parameter, Blas/Lapack is not used. In that case we prefer use naive code
   * that runs faster for small matrices.
   *
   * \sa getLapackMatrixMinSize()
   */
  static void setLapackMatrixMinSize(unsigned int min_size) { m_lapack_min_size = min_size; }
  //@}

  //-------------------------------------------------
  // Setting a diagonal matrix
  //-------------------------------------------------
  /** @name Setting a diagonal matrix  */
  //@{
  void diag(const float &val = 1.0);
  void diag(const vpColVectorf &A);
  // Initialize an identity matrix n-by-n
  void eye();
  void eye(unsigned int n);
  // Initialize an identity matrix m-by-n
  void eye(unsigned int m, unsigned int n);
  //@}

  //---------------------------------
  // Assignment
  //---------------------------------
  /** @name Assignment operators */
  //@{
  vpMatrixf &operator<<(float *);
  vpMatrixf &operator<<(float val);
  vpMatrixf &operator,(float val);
  vpMatrixf &operator=(const vpArray2D<float> &A);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrixf &operator=(const vpMatrixf &A);
  vpMatrixf &operator=(vpMatrixf &&A);

  vpMatrixf &operator=(const std::initializer_list<float> &list);
  vpMatrixf &operator=(const std::initializer_list<std::initializer_list<float> > &lists);
#endif
  vpMatrixf &operator=(float x);
  //@}

  //-------------------------------------------------
  // Stacking
  //-------------------------------------------------
  /** @name Stacking  */
  //@{
  // Stack the matrix A below the current one, copy if not initialized this =
  // [ this A ]^T
  void stack(const vpMatrixf &A);
  void stack(const vpRowVector &r);
  void stack(const vpColVectorf &c);
  // Stacks columns of a matrix in a vector
  void stackColumns(vpColVectorf &out);

  // Stacks columns of a matrix in a vector
  vpColVectorf stackColumns();

  // Stacks columns of a matrix in a vector
  void stackRows(vpRowVector &out);

  // Stacks columns of a matrix in a vector
  vpRowVector stackRows();
  //@}

  //---------------------------------
  // Matrix insertion
  //---------------------------------
  /** @name Matrix insertion */
  //@{
  // Insert matrix A in the current matrix at the given position (r, c).
  void insert(const vpMatrixf &A, unsigned int r, unsigned int c);
  //@}

  //-------------------------------------------------
  // Columns, Rows, Diag extraction, SubMatrix
  //-------------------------------------------------
  /** @name Columns, rows, sub-matrices extraction */
  //@{
  vpMatrixf extract(unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols) const;
  vpColVectorf getCol(unsigned int j) const;
  vpColVectorf getCol(unsigned int j, unsigned int i_begin, unsigned int size) const;
  vpRowVector getRow(unsigned int i) const;
  vpRowVector getRow(unsigned int i, unsigned int j_begin, unsigned int size) const;
  vpColVectorf getDiag() const;
  void init(const vpMatrixf &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols);
  //@}

  //---------------------------------
  // Matrix operations.
  //---------------------------------
  /** @name Matrix operations  */
  //@{
  // return the determinant of the matrix.
  float det(vpDetMethod method = LU_DECOMPOSITION) const;
  float detByLU() const;
#ifdef VISP_HAVE_EIGEN3
  float detByLUEigen3() const;
#endif
#if defined(VISP_HAVE_LAPACK)
  float detByLULapack() const;
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  float detByLUOpenCV() const;
#endif

  // Compute the exponential matrix of a square matrix
  vpMatrixf expm() const;

  // operation A = A + B
  vpMatrixf &operator+=(const vpMatrixf &B);
  // operation A = A - B
  vpMatrixf &operator-=(const vpMatrixf &B);
  vpMatrixf operator*(const vpMatrixf &B) const;
  vpMatrixf operator*(const vpRotationMatrix &R) const;
  vpMatrixf operator*(const vpHomogeneousMatrix &R) const;
  vpMatrixf operator*(const vpVelocityTwistMatrix &V) const;
  vpMatrixf operator*(const vpForceTwistMatrix &V) const;
  // operation t_out = A * t (A is unchanged, t and t_out are translation
  // vectors)
  vpTranslationVector operator*(const vpTranslationVector &tv) const;
  vpColVectorf operator*(const vpColVectorf &v) const;
  vpMatrixf operator+(const vpMatrixf &B) const;
  vpMatrixf operator-(const vpMatrixf &B) const;
  vpMatrixf operator-() const;

  //! Add x to all the element of the matrix : Aij = Aij + x
  vpMatrixf &operator+=(float x);
  //! subtract x to all the element of the matrix : Aij = Aij - x
  vpMatrixf &operator-=(float x);
  //! Multiply  all the element of the matrix by x : Aij = Aij * x
  vpMatrixf &operator*=(float x);
  //! Divide  all the element of the matrix by x : Aij = Aij / x
  vpMatrixf &operator/=(float x);

  // Cij = Aij * x (A is unchanged)
  vpMatrixf operator*(float x) const;
  // Cij = Aij / x (A is unchanged)
  vpMatrixf operator/(float x) const;

  /*!
    Return the sum of all the \f$a_{ij}\f$ elements of the matrix.

    \return Value of \f$\sum a_{ij}\f$
    */
  float sum() const;
  float sumSquare() const;

  //-------------------------------------------------
  // Hadamard product
  //-------------------------------------------------
  /** @name Hadamard product  */
  vpMatrixf hadamard(const vpMatrixf &m) const;

  //-------------------------------------------------
  // Kronecker product
  //-------------------------------------------------
  /** @name Kronecker product  */
  //@{
  // Compute Kronecker product matrix
  void kron(const vpMatrixf &m1, vpMatrixf &out) const;

  // Compute Kronecker product matrix
  vpMatrixf kron(const vpMatrixf &m1) const;
  //@}

  //-------------------------------------------------
  // Transpose
  //-------------------------------------------------
  /** @name Transpose  */
  //@{
  // Compute the transpose C = A^T
  vpMatrixf t() const;

  // Compute the transpose C = A^T
  vpMatrixf transpose() const;
  void transpose(vpMatrixf &At) const;

  vpMatrixf AAt() const;
  void AAt(vpMatrixf &B) const;

  vpMatrixf AtA() const;
  void AtA(vpMatrixf &B) const;
  //@}

  //-------------------------------------------------
  // Matrix inversion
  //-------------------------------------------------
  /** @name Matrix inversion  */
  //@{
  // inverse matrix A using the LU decomposition
  vpMatrixf inverseByLU() const;

#if defined(VISP_HAVE_EIGEN3)
  vpMatrixf inverseByLUEigen3() const;
#endif
#if defined(VISP_HAVE_LAPACK)
  vpMatrixf inverseByLULapack() const;
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpMatrixf inverseByLUOpenCV() const;
#endif

  // inverse matrix A using the Cholesky decomposition (only for real
  // symmetric matrices)
  vpMatrixf inverseByCholesky() const;

#if defined(VISP_HAVE_LAPACK)
  vpMatrixf inverseByCholeskyLapack() const;
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpMatrixf inverseByCholeskyOpenCV() const;
#endif

  // inverse matrix A using the QR decomposition
  vpMatrixf inverseByQR() const;
#if defined(VISP_HAVE_LAPACK)
  vpMatrixf inverseByQRLapack() const;
#endif

  // inverse triangular matrix
  vpMatrixf inverseTriangular(bool upper = true) const;

  vpMatrixf pseudoInverse(float svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrixf &Ap, float svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt) const;
  unsigned int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt,
                             vpMatrixf &kerAt) const;
  vpMatrixf pseudoInverse(int rank_in) const;
  int pseudoInverse(vpMatrixf &Ap, int rank_in) const;
  int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, int rank_in) const;
  int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, int rank_in, vpMatrixf &imA, vpMatrixf &imAt) const;
  int pseudoInverse(vpMatrixf &Ap, vpColVectorf &sv, int rank_in, vpMatrixf &imA, vpMatrixf &imAt, vpMatrixf &kerAt) const;

#if defined(VISP_HAVE_LAPACK)
  vpMatrixf pseudoInverseLapack(float svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrixf &Ap, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt,
                                   vpMatrixf &kerAt) const;
  vpMatrixf pseudoInverseLapack(int rank_in) const;
  int pseudoInverseLapack(vpMatrixf &Ap, int rank_in) const;
  int pseudoInverseLapack(vpMatrixf &Ap, vpColVectorf &sv, int rank_in) const;
  int pseudoInverseLapack(vpMatrixf &Ap, vpColVectorf &sv, int rank_in, vpMatrixf &imA, vpMatrixf &imAt,
                          vpMatrixf &kerAt) const;
#endif
#if defined(VISP_HAVE_EIGEN3)
  vpMatrixf pseudoInverseEigen3(float svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrixf &Ap, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt,
                                   vpMatrixf &kerAt) const;
  vpMatrixf pseudoInverseEigen3(int rank_in) const;
  int pseudoInverseEigen3(vpMatrixf &Ap, int rank_in) const;
  int pseudoInverseEigen3(vpMatrixf &Ap, vpColVectorf &sv, int rank_in) const;
  int pseudoInverseEigen3(vpMatrixf &Ap, vpColVectorf &sv, int rank_in, vpMatrixf &imA, vpMatrixf &imAt,
                          vpMatrixf &kerAt) const;
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpMatrixf pseudoInverseOpenCV(float svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrixf &Ap, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt,
                                   vpMatrixf &kerAt) const;
  vpMatrixf pseudoInverseOpenCV(int rank_in) const;
  int pseudoInverseOpenCV(vpMatrixf &Ap, int rank_in) const;
  int pseudoInverseOpenCV(vpMatrixf &Ap, vpColVectorf &sv, int rank_in) const;
  int pseudoInverseOpenCV(vpMatrixf &Ap, vpColVectorf &sv, int rank_in, vpMatrixf &imA, vpMatrixf &imAt,
                          vpMatrixf &kerAt) const;
#endif
  //@}

  //-------------------------------------------------
  // SVD decomposition
  //-------------------------------------------------

  /** @name SVD decomposition  */
  //@{
  float cond(float svThreshold = 1e-6) const;
  unsigned int kernel(vpMatrixf &kerAt, float svThreshold = 1e-6) const;
  unsigned int nullSpace(vpMatrixf &kerA, float svThreshold = 1e-6) const;
  unsigned int nullSpace(vpMatrixf &kerA, int dim) const;

  // solve Ax=B using the SVD decomposition (usage A = solveBySVD(B,x) )
  void solveBySVD(const vpColVectorf &B, vpColVectorf &x) const;
  // solve Ax=B using the SVD decomposition (usage  x=A.solveBySVD(B))
  vpColVectorf solveBySVD(const vpColVectorf &B) const;

  // singular value decomposition SVD
  void svd(vpColVectorf &w, vpMatrixf &V);
#ifdef VISP_HAVE_EIGEN3
  void svdEigen3(vpColVectorf &w, vpMatrixf &V);
#endif
#if defined(VISP_HAVE_LAPACK)
  void svdLapack(vpColVectorf &w, vpMatrixf &V);
#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  void svdOpenCV(vpColVectorf &w, vpMatrixf &V);
#endif
  //@}

  //-------------------------------------------------
  // QR decomposition
  //-------------------------------------------------

  /** @name QR decomposition  */
  //@{
  unsigned int qr(vpMatrixf &Q, vpMatrixf &R, bool full = false, bool squareR = false, float tol = 1e-6) const;
  unsigned int qrPivot(vpMatrixf &Q, vpMatrixf &R, vpMatrixf &P, bool full = false, bool squareR = false,
                       float tol = 1e-6) const;
  void solveByQR(const vpColVectorf &b, vpColVectorf &x) const;
  vpColVectorf solveByQR(const vpColVectorf &b) const;
  //@}

  //-------------------------------------------------
  // Eigen values and vectors
  //-------------------------------------------------
  /** @name Eigen values  */

  //@{
  // compute the eigen values using Lapack
  vpColVectorf eigenValues() const;
  void eigenValues(vpColVectorf &evalue, vpMatrixf &evector) const;
  //@}

  //-------------------------------------------------
  // Norms
  //-------------------------------------------------
  /** @name Norms  */
  //@{
  float euclideanNorm() const;
  float frobeniusNorm() const;
  float inducedL2Norm() const;
  float infinityNorm() const;
  //@}

  //---------------------------------
  // Printing
  //---------------------------------
  /** @name Printing  */
  //@{
  std::ostream &cppPrint(std::ostream &os, const std::string &matrixName = "A", bool octet = false) const;
  std::ostream &csvPrint(std::ostream &os) const;
  std::ostream &maplePrint(std::ostream &os) const;
  std::ostream &matlabPrint(std::ostream &os) const;
  int print(std::ostream &s, unsigned int length, const std::string &intro = "") const;
  void printSize() const { std::cout << getRows() << " x " << getCols() << "  "; }
  //@}

  //------------------------------------------------------------------
  // Static functionalities
  //------------------------------------------------------------------

  //---------------------------------
  // Setting a diagonal matrix with Static Public Member Functions
  //---------------------------------
  /** @name Setting a diagonal matrix with Static Public Member Functions  */
  //@{
  // Create a diagonal matrix with the element of a vector DAii = Ai
  static void createDiagonalMatrix(const vpColVectorf &A, vpMatrixf &DA);
  //@}

  //---------------------------------
  // Matrix insertion with Static Public Member Functions
  //---------------------------------
  /** @name Matrix insertion with Static Public Member Functions  */
  //@{
  // Insert matrix B in matrix A at the given position (r, c).
  static vpMatrixf insert(const vpMatrixf &A, const vpMatrixf &B, unsigned int r, unsigned int c);
  // Insert matrix B in matrix A (not modified) at the given position (r, c),
  // the result is given in matrix C.
  static void insert(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C, unsigned int r, unsigned int c);

  //---------------------------------
  // Stacking with Static Public Member Functions
  //---------------------------------
  /** @name Stacking with Static Public Member Functions  */
  //@{
  // Juxtapose to matrices C = [ A B ]
  static vpMatrixf juxtaposeMatrices(const vpMatrixf &A, const vpMatrixf &B);
  // Juxtapose to matrices C = [ A B ]
  static void juxtaposeMatrices(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C);
  // Stack two matrices C = [ A B ]^T
  static vpMatrixf stack(const vpMatrixf &A, const vpMatrixf &B);
  static vpMatrixf stack(const vpMatrixf &A, const vpRowVector &r);
  static vpMatrixf stack(const vpMatrixf &A, const vpColVectorf &c);

  // Stack two matrices C = [ A B ]^T
  static void stack(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C);
  static void stack(const vpMatrixf &A, const vpRowVector &r, vpMatrixf &C);
  static void stack(const vpMatrixf &A, const vpColVectorf &c, vpMatrixf &C);
  //@}

  //---------------------------------
  // Matrix operations Static Public Member Functions
  //---------------------------------
  /** @name Matrix operations with Static Public Member Functions  */
  //@{
  static void add2Matrices(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C);
  static void add2Matrices(const vpColVectorf &A, const vpColVectorf &B, vpColVectorf &C);
  static void add2WeightedMatrices(const vpMatrixf &A, const float &wA, const vpMatrixf &B, const float &wB,
                                   vpMatrixf &C);
  static void computeHLM(const vpMatrixf &H, const float &alpha, vpMatrixf &HLM);
  static void mult2Matrices(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C);
  static void mult2Matrices(const vpMatrixf &A, const vpMatrixf &B, vpRotationMatrix &C);
  static void mult2Matrices(const vpMatrixf &A, const vpMatrixf &B, vpHomogeneousMatrix &C);
  static void mult2Matrices(const vpMatrixf &A, const vpColVectorf &B, vpColVectorf &C);
  static void multMatrixVector(const vpMatrixf &A, const vpColVectorf &v, vpColVectorf &w);
  static void negateMatrix(const vpMatrixf &A, vpMatrixf &C);
  static void sub2Matrices(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C);
  static void sub2Matrices(const vpColVectorf &A, const vpColVectorf &B, vpColVectorf &C);
  //@}

  //---------------------------------
  // Kronecker product Static Public Member Functions
  //---------------------------------
  /** @name Kronecker product with Static Public Member Functions  */
  //@{
  // Compute Kronecker product matrix
  static void kron(const vpMatrixf &m1, const vpMatrixf &m2, vpMatrixf &out);

  // Compute Kronecker product matrix
  static vpMatrixf kron(const vpMatrixf &m1, const vpMatrixf &m2);
  //@}

  //-------------------------------------------------
  // 2D Convolution Static Public Member Functions
  //-------------------------------------------------
  /** @name 2D Convolution with Static Public Member Functions  */
  static vpMatrixf conv2(const vpMatrixf &M, const vpMatrixf &kernel, const std::string &mode = "full");
  static void conv2(const vpMatrixf &M, const vpMatrixf &kernel, vpMatrixf &res, const std::string &mode = "full");

  //---------------------------------
  // Covariance computation Static Public Member Functions
  //---------------------------------
  /** @name Covariance computation with Static Public Member Functions  */
  //@{
  static vpMatrixf computeCovarianceMatrix(const vpMatrixf &A, const vpColVectorf &x, const vpColVectorf &b);
  static vpMatrixf computeCovarianceMatrix(const vpMatrixf &A, const vpColVectorf &x, const vpColVectorf &b,
                                          const vpMatrixf &w);
  static vpMatrixf computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVectorf &deltaS,
                                             const vpMatrixf &Ls, const vpMatrixf &W);
  static vpMatrixf computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVectorf &deltaS,
                                             const vpMatrixf &Ls);
  //@}

  //---------------------------------
  // Matrix I/O  Static Public Member Functions
  //---------------------------------
  /** @name Matrix I/O with Static Public Member Functions  */
  //@{
  /*!
    Load a matrix from a file. This function overloads vpArray2D::load().

    \param filename : Absolute file name.
    \param M : Matrix to be loaded.
    \param binary : If true the matrix data are considered as binary, otherwise as human readable (text) data. Using
    binary data allows to keep data precision.
    \param header : Header of the file is loaded in this parameter.

    \return Returns true if success, false otherwise.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrixf.h>

    int main()
    {
      std::string filename("matrix.bin");
      bool binary_data = true;
      {
        vpMatrixf M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrixf::saveMatrix(filename, M, binary_data, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrixf N;
        char header[FILENAME_MAX];
        if (vpMatrixf::loadMatrix(filename, N, binary_data, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.bin` file where data are saved as binary data is the following:
    \verbatim
    % cat matrix.bin
    My header??@@@%
    \endverbatim

    \sa saveMatrix(), saveMatrixYAML(), loadMatrixYAML()
  */
  static inline bool loadMatrix(const std::string &filename, vpArray2D<float> &M, bool binary = false,
                                char *header = NULL)
  {
    return vpArray2D<float>::load(filename, M, binary, header);
  }

  /*!
    Load a matrix from a YAML-formatted file. This function overloads
    vpArray2D::loadYAML().

    \param filename : Absolute YAML file name.
    \param M : Matrix to be loaded from the file.
    \param header : Header of the file is loaded in this parameter.

    \return Returns true when success, false otherwise.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrixf.h>

    int main()
    {
      std::string filename("matrix.yaml");
      {
        vpMatrixf M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrixf::saveMatrixYAML(filename, M, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrixf N;
        char header[FILENAME_MAX];
        if (vpMatrixf::loadMatrixYAML(filename, N, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.yaml` file is the following:
    \verbatim
    % cat matrix.yaml
    My header
    rows: 2
    cols: 3
    data:
      - [-1, -2, -3]
      - [4, 5.5, 6]
    \endverbatim

    \sa saveMatrixYAML(), saveMatrix(), loadMatrix()
  */
  static inline bool loadMatrixYAML(const std::string &filename, vpArray2D<float> &M, char *header = NULL)
  {
    return vpArray2D<float>::loadYAML(filename, M, header);
  }

  /*!
    Save a matrix to a file. This function overloads vpArray2D::save().

    \param filename : Absolute file name.
    \param M : Matrix to be saved.
    \param binary : If true the matrix is save as a binary file, otherwise as a text file.
    \param header : Optional line that will be saved at the beginning of the file as a header.

    \return Returns true if no problem appends.

    \warning If you save the matrix as a text file the precision is less
    than if you save it as a binary file.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrixf.h>

    int main()
    {
      std::string filename("matrix.bin");
      bool binary_data = true;
      {
        vpMatrixf M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrixf::saveMatrix(filename, M, binary_data, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrixf N;
        char header[FILENAME_MAX];
        if (vpMatrixf::loadMatrix(filename, N, binary_data, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.bin` file where data are saved as binary data is the following:
    \verbatim
    % cat matrix.bin
    My header??@@@%
    \endverbatim

    \sa loadMatrix(), saveMatrixYAML(), loadMatrixYAML()
  */
  static inline bool saveMatrix(const std::string &filename, const vpArray2D<float> &M, bool binary = false,
                                const char *header = "")
  {
    return vpArray2D<float>::save(filename, M, binary, header);
  }

  /*!
    Save a matrix in a YAML-formatted file. This function overloads
    vpArray2D::saveYAML().

    \param filename : Absolute file name.
    \param M : Matrix to be saved in the file.
    \param header : Optional lines that will be saved at the beginning of the
    file as a header.

    \return Returns true if success.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrixf.h>

    int main()
    {
      std::string filename("matrix.yaml");
      {
        vpMatrixf M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrixf::saveMatrixYAML(filename, M, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrixf N;
        char header[FILENAME_MAX];
        if (vpMatrixf::loadMatrixYAML(filename, N, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.yaml` file is the following:
    \verbatim
    % cat matrix.yaml
    My header
    rows: 2
    cols: 3
    data:
      - [-1, -2, -3]
      - [4, 5.5, 6]
    \endverbatim

    \sa saveMatrix(), loadMatrix(), loadMatrixYAML()
  */
  static inline bool saveMatrixYAML(const std::string &filename, const vpArray2D<float> &M, const char *header = "")
  {
    return vpArray2D<float>::saveYAML(filename, M, header);
  }
  //@}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Only provided for compatibilty with ViSP previous releases.
     This function does nothing.
   */
  vp_deprecated void init() {}

  /*!
     \deprecated You should rather use stack(const vpMatrixf &A)
   */
  vp_deprecated void stackMatrices(const vpMatrixf &A) { stack(A); }
  /*!
     \deprecated You should rather use stack(const vpMatrixf &A, const vpMatrixf
     &B)
   */
  vp_deprecated static vpMatrixf stackMatrices(const vpMatrixf &A, const vpMatrixf &B) { return stack(A, B); }
  /*!
     \deprecated You should rather use stack(const vpMatrixf &A, const vpMatrixf
     &B, vpMatrixf &C)
   */
  vp_deprecated static void stackMatrices(const vpMatrixf &A, const vpMatrixf &B, vpMatrixf &C) { stack(A, B, C); }
  /*!
     \deprecated You should rather use stack(const vpMatrixf &A, const vpMatrixf
     &B)
   */
  vp_deprecated static vpMatrixf stackMatrices(const vpMatrixf &A, const vpRowVector &B);
  /*!
     \deprecated You should rather use stack(const vpMatrixf &A, const
     vpRowVector &B, vpMatrixf &C)
   */
  vp_deprecated static void stackMatrices(const vpMatrixf &A, const vpRowVector &B, vpMatrixf &C);
  /*!
     \deprecated You should rather use vpColVectorf::stack(const vpColVectorf
     &A, const vpColVectorf &B)
   */
  vp_deprecated static vpMatrixf stackMatrices(const vpColVectorf &A, const vpColVectorf &B);
  /*!
     \deprecated You should rather use vpColVectorf::stack(const vpColVectorf
     &A, const vpColVectorf &B, vpColVectorf &C)
   */
  vp_deprecated static void stackMatrices(const vpColVectorf &A, const vpColVectorf &B, vpColVectorf &C);

  /*!
     \deprecated You should rather use diag(const float &)
   */
  vp_deprecated void setIdentity(const float &val = 1.0);

  vp_deprecated vpRowVector row(unsigned int i);
  vp_deprecated vpColVectorf column(unsigned int j);

  // Deprecated functions using GSL
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!
     \deprecated You should rather use detByLULapack() or detByLU().
   */
  vp_deprecated float detByLUGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return detByLULapack();
#else
    throw(vpException(vpException::fatalError, "Undefined detByLULapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByLULapack() or inverseByLU().
   */
  vp_deprecated vpMatrixf inverseByLUGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByLULapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByLULapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByCholeskyLapack() or inverseByCholesky().
   */
  vpMatrixf inverseByCholeskyGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByCholeskyLapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByCholeskyLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByQRLapack() or inverseByQR().
   */
  vpMatrixf inverseByQRGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByQRLapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByQRLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  vpMatrixf pseudoInverseGsl(float svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(svThreshold);
#else
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  unsigned int pseudoInverseGsl(vpMatrixf &Ap, float svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, svThreshold);
#else
    (void)Ap;
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  unsigned int pseudoInverseGsl(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, sv, svThreshold);
#else
    (void)Ap;
    (void)sv;
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  unsigned int pseudoInverseGsl(vpMatrixf &Ap, vpColVectorf &sv, float svThreshold, vpMatrixf &imA, vpMatrixf &imAt,
                                vpMatrixf &kerAt) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, sv, svThreshold, imA, imAt, kerAt);
#else
    (void)Ap;
    (void)sv;
    (void)svThreshold;
    (void)imA;
    (void)imAt;
    (void)kerAt;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use svdLapack() or svd().
   */
  void svdGsl(vpColVectorf &w, vpMatrixf &V)
  {
#if defined(VISP_HAVE_LAPACK)
    svdLapack(w, V);
#else
    (void)w;
    (void)V;
    throw(vpException(vpException::fatalError, "Undefined svdLapack(). Install Lapack 3rd party"));
#endif
  }

#endif // ifndef DOXYGEN_SHOULD_SKIP_THIS
  //@}
#endif

private:
  static unsigned int m_lapack_min_size;
  static const unsigned int m_lapack_min_size_default;

#if defined(VISP_HAVE_LAPACK)
  static void blas_dgemm(char trans_a, char trans_b, unsigned int M_, unsigned int N_, unsigned int K_, float alpha,
                         float *a_data, unsigned int lda_, float *b_data, unsigned int ldb_, float beta,
                         float *c_data, unsigned int ldc_);
  static void blas_dgemv(char trans, unsigned int M_, unsigned int N_, float alpha, float *a_data, unsigned int lda_,
                         float *x_data, int incx_, float beta, float *y_data, int incy_);
  static void blas_dsyev(char jobz, char uplo, unsigned int n_, float *a_data, unsigned int lda_, float *w_data,
                         float *work_data, int lwork_, int &info_);
#endif

  static void computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVectorf &deltaS, const vpMatrixf &Ls,
                                         vpMatrixf &Js, vpColVectorf &deltaP);
};

//////////////////////////////////////////////////////////////////////////
#if defined(VISP_USE_MSVC) && defined(visp_EXPORTS)
const __declspec(selectany) unsigned int vpMatrixf::m_lapack_min_size_default = 0;
__declspec(selectany) unsigned int vpMatrixf::m_lapack_min_size = vpMatrixf::m_lapack_min_size_default;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpMatrixf operator*(const float &x, const vpMatrixf &A);
#endif
