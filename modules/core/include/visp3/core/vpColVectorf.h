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
 * Provide some simple operation on column vectors.
 *
 * Authors:
 * Eric Marchand
 *
*****************************************************************************/

#ifndef _vpColVectorf_h_
#define _vpColVectorf_h_

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRowVector.h>

class vpMatrix;
class vpRowVector;
class vpRotationVector;
class vpTranslationVector;
class vpPoseVector;

/*!
  \file vpColVectorf.h
  \brief definition of column vector class as well
  as a set of operations on these vector
*/

/*!
  \class vpColVectorf
  \ingroup group_core_matrices

  \brief Implementation of column vector and the associated operations.

  This class provides a data structure for a column vector that contains
  values of float. It contains also some functions to achieve a set of
  operations on these vectors.

  The vpColVectorf class is derived from vpArray2D<float>.

  The code below shows how to create a 3-element column vector of floats, set the element values and access them:
  \code
  #include <visp3/code/vpColVectorf.h

  int main()
  {
    vpColVectorf v(3);
    v[0] = -1; v[1] = -2.1; v[2] = -3;

    std::cout << "v:" << std::endl;
    for (unsigned int i = 0; i < v.size(); i++) {
      std::cout << v[i] << std::endl;
    }
  }
  \endcode
  Once build, this previous code produces the following output:
  \code{.unparsed}
  v:
  -1
  -2.1
  -3
  \endcode
  You can also use operator<< to initialize a column vector as previously:
  \code
  #include <visp3/code/vpColVectorf.h

  int main()
  {
    vpColVectorf v;
    v << -1, -2.1, -3;
    std::cout << "v:" << v << std::endl;
  }
  \endcode

  If ViSP is build with c++11 enabled, you can do the same using:
  \code
  #include <visp3/code/vpColVectorf.h

  int main()
  {
  #if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpColVectorf v({-1, -2.1, -3});
    std::cout << "v:\n" << v << std::endl;
  #endif
  }
  \endcode
  The vector could also be initialized using operator=(const std::initializer_list< float > &)
  \code
  int main()
  {
  #if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpColVectorf v;
    v = {-1, -2.1, -3};
  #endif
  }
  \endcode

  <b>JSON serialization</b>

  Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpColVectorf.
  The following sample code shows how to save a pose vector in a file named `col-vector.json`
  and reload the values from this JSON file.
  \code
  #include <visp3/core/vpColVectorf.h>

  int main()
  {
  #if defined(VISP_HAVE_NLOHMANN_JSON)
    std::string filename = "col-vector.json";
    {
      vpColVectorf v({ 1, 2, 3, 4 });
      std::ofstream file(filename);
      const nlohmann::json j = v;
      file << j;
      file.close();
    }
    {
      std::ifstream file(filename);
      const nlohmann::json j = nlohmann::json::parse(file);
      vpColVectorf v;
      v = j.get<vpColVectorf>();
      file.close();
      std::cout << "Read homogeneous matrix from " << filename << ":\n" << v.t() << std::endl;
    }
  #endif
  }
  \endcode
  If you build and execute the sample code, it will produce the following output:
  \code{.unparsed}
  Read homogeneous matrix from col-vector.json:
  1  2  3  4
  \endcode

  The content of the `pose-vector.json` file is the following:
  \code{.unparsed}
  $ cat col-vector.json
  {"cols":1,"data":[1.0,2.0,3.0,4.0],"rows":4,"type":"vpColVectorf"}
  \endcode
*/
class VISP_EXPORT vpColVectorf : public vpArray2D<float>
{
  friend class vpMatrix;

public:
  //! Basic constructor that creates an empty 0-size column vector.
  vpColVectorf() : vpArray2D<float>() { }
  //! Construct a column vector of size n. \warning Elements are not
  //! initialized. If you want to set an initial value use
  //! vpColVectorf(unsigned int, float).
  explicit vpColVectorf(unsigned int n) : vpArray2D<float>(n, 1) { }
  //! Construct a column vector of size n. Each element is set to \e val.
  vpColVectorf(unsigned int n, float val) : vpArray2D<float>(n, 1, val) { }
  //! Copy constructor that allows to construct a column vector from an other
  //! one.
  vpColVectorf(const vpColVectorf &v) : vpArray2D<float>(v) { }
  vpColVectorf(const vpColVectorf &v, unsigned int r, unsigned int nrows);
  //! Constructor that initialize a column vector from a 3-dim (Euler or
  //! \f$\theta {\bf u}\f$) or 4-dim (quaternion) rotation vector.
  vpColVectorf(const vpRotationVector &v);
  //! Constructor that initialize a column vector from a 6-dim pose vector.
  vpColVectorf(const vpPoseVector &p);
  //! Constructor that initialize a column vector from a 3-dim translation
  //! vector.
  vpColVectorf(const vpTranslationVector &t);
  vpColVectorf(const vpMatrix &M);
  vpColVectorf(const vpMatrix &M, unsigned int j);
  vpColVectorf(const std::vector<double> &v);
  vpColVectorf(const std::vector<float> &v);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpColVectorf(vpColVectorf &&v);
  vpColVectorf(const std::initializer_list<float> &list) : vpArray2D<float>(static_cast<unsigned int>(list.size()), 1)
  {
    std::copy(list.begin(), list.end(), data);
  }
#endif
  /*!
    Destructor.
  */
  virtual ~vpColVectorf() { }

  /*!
    Removes all elements from the vector (which are destroyed),
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

  std::ostream &cppPrint(std::ostream &os, const std::string &matrixName = "A", bool octet = false) const;
  std::ostream &csvPrint(std::ostream &os) const;

  /*!
    Converts a column vector containing angles in degrees into radians and returns a reference to the vector.
    \return A reference to the vector with values expressed in [rad].
    \sa rad2deg()
  */
  inline vpColVectorf &deg2rad()
  {
    float d2r = M_PI / 180.0;

    (*this) *= d2r;
    return (*this);
  }

  vp_deprecated float euclideanNorm() const;
  /*!
     Extract a sub-column vector from a column vector.
     \param r : Index of the row corresponding to the first element of the
     vector to extract. \param colsize : Size of the vector to extract.
     \exception vpException::fatalError If the vector to extract is not
     contained in the original one.

     \code
     vpColVectorf v1;
     for (unsigned int i=0; i<4; i++)
       v1.stack(i);
     // v1 is equal to [0 1 2 3]^T
     vpColVectorf v2 = v1.extract(1, 3);
     // v2 is equal to [1 2 3]^T
     \endcode
   */
  vpColVectorf extract(unsigned int r, unsigned int colsize) const
  {
    if (r >= rowNum || r + colsize > rowNum) {
      throw(vpException(vpException::fatalError,
        "Cannot extract a (%dx1) column vector from a (%dx1) "
        "column vector starting at index %d",
        colsize, rowNum, r));
    }

    return vpColVectorf(*this, r, colsize);
  }

  float frobeniusNorm() const;
  vpColVectorf hadamard(const vpColVectorf &v) const;

  float infinityNorm() const;
  void init(const vpColVectorf &v, unsigned int r, unsigned int nrows);
  void insert(unsigned int i, const vpColVectorf &v);
  void insert(const vpColVectorf &v, unsigned int i);

  std::ostream &maplePrint(std::ostream &os) const;
  std::ostream &matlabPrint(std::ostream &os) const;

  vpColVectorf &normalize();
  vpColVectorf &normalize(vpColVectorf &x) const;

  //! Operator that allows to set a value of an element \f$v_i\f$: v[i] = x
  inline float &operator[](unsigned int n) { return *(data + n); }
  //! Operator that allows to get the value of an element \f$v_i\f$: x = v[i]
  inline const float &operator[](unsigned int n) const { return *(data + n); }
  //! Copy operator.   Allow operation such as A = v
  vpColVectorf &operator=(const vpColVectorf &v);
  vpColVectorf &operator=(const vpPoseVector &p);
  vpColVectorf &operator=(const vpRotationVector &rv);
  vpColVectorf &operator=(const vpTranslationVector &tv);
  vpColVectorf &operator=(const vpMatrix &M);
  vpColVectorf &operator=(const std::vector<double> &v);
  vpColVectorf &operator=(const std::vector<float> &v);
  vpColVectorf &operator=(float x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpColVectorf &operator=(vpColVectorf &&v);
  vpColVectorf &operator=(const std::initializer_list<float> &list);
#endif
  //! Comparison operator.
  bool operator==(const vpColVectorf &v) const;
  bool operator==(float v) const;
  bool operator!=(const vpColVectorf &v) const;
  bool operator!=(float v) const;

  float operator*(const vpColVectorf &x) const;
  vpMatrix operator*(const vpRowVector &v) const;
  vpColVectorf operator*(float x) const;
  vpColVectorf &operator*=(float x);

  vpColVectorf operator/(float x) const;
  vpColVectorf &operator/=(float x);

  vpColVectorf operator+(const vpColVectorf &v) const;
  vpTranslationVector operator+(const vpTranslationVector &t) const;
  vpColVectorf &operator+=(vpColVectorf v);

  vpColVectorf operator-(const vpColVectorf &v) const;
  vpColVectorf &operator-=(vpColVectorf v);
  vpColVectorf operator-() const;

  vpColVectorf &operator<<(const vpColVectorf &v);
  vpColVectorf &operator<<(float *);
  vpColVectorf &operator<<(float val);
  vpColVectorf &operator,(float val);

  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
    Converts a column vector containing angles in radians into degrees and returns a reference to the vector.
    \return A reference to the vector with values expressed in [deg].
    \sa deg2rad()
  */
  inline vpColVectorf &rad2deg()
  {
    float r2d = 180.0 / M_PI;

    (*this) *= r2d;
    return (*this);
  }

  void reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols);
  vpMatrix reshape(unsigned int nrows, unsigned int ncols);

  /*! Modify the size of the column vector.
    \param i : Size of the vector. This value corresponds to the vector number
    of rows.
    \param flagNullify : If true, set the data to zero.
    \exception vpException::fatalError When \e ncols is not equal to 1.
   */
  void resize(unsigned int i, bool flagNullify = true) { vpArray2D<float>::resize(i, 1, flagNullify); }
  /*!
    Resize the column vector to a \e nrows-dimension vector.
    This function can only be used with \e ncols = 1.
    \param nrows : Vector number of rows. This value corresponds
    to the size of the vector.
    \param ncols : Vector number of columns. This value should be set to 1.
    \param flagNullify : If true, set the data to zero.
    \exception vpException::fatalError When \e ncols is not equal to 1.

    */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify)
  {
    if (ncols != 1) {
      throw(vpException(vpException::fatalError,
        "Cannot resize a column vector to a (%dx%d) "
        "dimension vector that has more than one column",
        nrows, ncols));
    }
    vpArray2D<float>::resize(nrows, ncols, flagNullify);
  }

  void stack(float d);
  void stack(const vpColVectorf &v);

  float sum() const;
  float sumSquare() const;
  vpRowVector t() const;
  std::vector<float> toStdVector() const;
  vpRowVector transpose() const;
  void transpose(vpRowVector &v) const;

  /*!
     Compute and return the cross product of two 3-dimension vectors: \f$a
     \times b\f$. \param a : 3-dimension column vector. \param b : 3-dimension
     column vector. \return The cross product \f$a \times b\f$.

     \exception vpException::dimensionError If the vectors dimension is not
     equal to 3.
   */
  inline static vpColVectorf cross(const vpColVectorf &a, const vpColVectorf &b) { return crossProd(a, b); }
  static vpColVectorf crossProd(const vpColVectorf &a, const vpColVectorf &b);

  static float dotProd(const vpColVectorf &a, const vpColVectorf &b);
  static vpColVectorf invSort(const vpColVectorf &v);
  static float median(const vpColVectorf &v);
  static float mean(const vpColVectorf &v);
  // Compute the skew matrix [v]x
  static vpMatrix skew(const vpColVectorf &v);

  static vpColVectorf sort(const vpColVectorf &v);

  static vpColVectorf stack(const vpColVectorf &A, const vpColVectorf &B);
  static void stack(const vpColVectorf &A, const vpColVectorf &B, vpColVectorf &C);

  static float stdev(const vpColVectorf &v, bool useBesselCorrection = false);

#ifdef VISP_HAVE_NLOHMANN_JSON
  friend void to_json(nlohmann::json &j, const vpColVectorf &cam);
  friend void from_json(const nlohmann::json &j, vpColVectorf &cam);
#endif

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  vp_deprecated void init() { }
  /*!
     \deprecated You should rather use extract().
   */
  vp_deprecated vpColVectorf rows(unsigned int first_row, unsigned int last_row) const
  {
    return vpColVectorf(*this, first_row - 1, last_row - first_row + 1);
  }
  /*!
     \deprecated You should rather use eye()
   */
  vp_deprecated void setIdentity(const float &val = 1.0);
  /*!
     \deprecated You should rather use stack(const vpColVectorf &)
   */
  vp_deprecated void stackMatrices(const vpColVectorf &r) { stack(r); }
  /*!
     \deprecated You should rather use stack(const vpColVectorf &A, const
     vpColVectorf &B)
   */
  vp_deprecated static vpColVectorf stackMatrices(const vpColVectorf &A, const vpColVectorf &B) { return stack(A, B); }
  /*!
     \deprecated You should rather use stack(const vpColVectorf &A, const
     vpColVectorf &B, vpColVectorf &C)
   */
  vp_deprecated static void stackMatrices(const vpColVectorf &A, const vpColVectorf &B, vpColVectorf &C)
  {
    stack(A, B, C);
  }

  vp_deprecated void insert(const vpColVectorf &v, unsigned int r, unsigned int c = 0);
  //@}
#endif
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpColVectorf operator*(const float &x, const vpColVectorf &v);

#ifdef VISP_HAVE_NLOHMANN_JSON
inline void to_json(nlohmann::json &j, const vpColVectorf &v)
{
  const vpArray2D<float> *asArray = (vpArray2D<float>*) & v;
  to_json(j, *asArray);
  j["type"] = "vpColVectorf";
}
inline void from_json(const nlohmann::json &j, vpColVectorf &v)
{
  vpArray2D<float> *asArray = (vpArray2D<float>*) & v;
  from_json(j, *asArray);
  if (v.getCols() != 1) {
    throw vpException(vpException::badValue, "From JSON, tried to read a 2D array into a vpColVectorf");
  }
}

#endif
#endif
