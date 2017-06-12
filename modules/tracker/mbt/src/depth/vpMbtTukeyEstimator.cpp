#include <iostream>
#include <cmath>
#include <algorithm>

#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbtTukeyEstimator.h>

#define USE_TRANSFORM 1
#if defined(VISP_HAVE_CPP11_COMPATIBILITY) && USE_TRANSFORM
#  define HAVE_TRANSFORM 1
#endif

#if HAVE_TRANSFORM
#  include <functional>
#endif

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
#  endif
#endif

#define USE_ORIGINAL_TUKEY_CODE 1


#if HAVE_TRANSFORM
namespace {
  template <typename T>
  struct AbsDiff : public std::binary_function<T, T, T> {
    double operator()(const T a, const T b) const {
      return std::fabs(a - b);
    }
  };
}
#endif

template class vpMbtTukeyEstimator<float>;
template class vpMbtTukeyEstimator<double>;

template <typename T> T vpMbtTukeyEstimator<T>::getMedian(std::vector<T> &vec) {
  //Not the exact median when even number of elements
  int index = (int) ( ceil(vec.size()/2.0) ) - 1;
  std::nth_element(vec.begin(), vec.begin() + index, vec.end());
  return vec[index];
}

#if VISP_HAVE_SSSE3
namespace {
  inline __m128 abs_ps(__m128 x) {
    static const __m128 sign_mask = _mm_set1_ps(-0.f); // -0.f = 1 << 31
    return _mm_andnot_ps(sign_mask, x);
  }
}

template <>
void vpMbtTukeyEstimator<float>::MEstimator(const std::vector<float> &residues, std::vector<float> &weights, const float NoiseThreshold) {
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  float med = getMedian(m_residues);
  m_normres.resize(residues.size());

  size_t i = 0;
  __m128 med_128 = _mm_set_ps1(med);

  if (m_residues.size() >= 4) {
    for (i = 0; i <= m_residues.size()-4; i+=4) {
      __m128 residues_128 = _mm_loadu_ps(residues.data()+i);
      _mm_storeu_ps( m_normres.data()+i, abs_ps( _mm_sub_ps(residues_128, med_128) ) );
    }
  }

  for (; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i]- med));
  }

  m_residues = m_normres;
  float normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  float sigma = 1.4826f*normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}

template <>
void vpMbtTukeyEstimator<double>::MEstimator(const std::vector<double> &residues, std::vector<double> &weights, const double NoiseThreshold) {
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  float med = getMedian(m_residues);
  m_normres.resize(residues.size());

#if HAVE_TRANSFORM
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<double>(), std::placeholders::_1, med));
#else
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i]- med));
  }
#endif

  m_residues = m_normres;
  double normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  float sigma = 1.4826*normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}
#else
template <typename T>
void vpMbtTukeyEstimator<T>::MEstimator(const std::vector<T> &residues, std::vector<T> &weights, const T NoiseThreshold) {
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  T med = getMedian(m_residues);
  m_normres.resize(residues.size());

#if HAVE_TRANSFORM
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<T>(), std::placeholders::_1, med));
#else
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i]- med));
  }
#endif

  m_residues = m_normres;
  T normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  T sigma = 1.4826*normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}
#endif

template <>
void vpMbtTukeyEstimator<double>::psiTukey(const double sig, std::vector<double> &x, vpColVector &weights) {
  double cst_const = 4.6851;
  double inv_cst_const = 1 / 4.6851;
  double inv_sig = 1 / sig;

  for(size_t i = 0; i < x.size(); i++) {
#if USE_ORIGINAL_TUKEY_CODE
    if (std::fabs(sig) <= std::numeric_limits<double>::epsilon()
        && std::fabs(weights[i]) > std::numeric_limits<double>::epsilon() //sig should be equal to 0 only if NoiseThreshold == 0
        ) {
      weights[i] = 1;
      continue;
    }
#endif

    double xi_sig = x[i] * inv_sig;

    if ((std::fabs(xi_sig) <= cst_const)
#if USE_ORIGINAL_TUKEY_CODE
        && std::fabs(weights[i]) > std::numeric_limits<double>::epsilon() //Consider the previous weights here
#endif
        ) {
      weights[i] = ( 1 - (xi_sig * inv_cst_const)*(xi_sig * inv_cst_const) ) * ( 1 - (xi_sig * inv_cst_const)*(xi_sig * inv_cst_const) ); //vpMath::sqr( 1 - vpMath::sqr(xi_sig * inv_cst_const) );
    } else {
      //Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}

template <>
void vpMbtTukeyEstimator<double>::MEstimator(const vpColVector &residues, vpColVector &weights, const double NoiseThreshold) {
  if (residues.size() == 0) {
    return;
  }

  m_residues.resize(0);
  m_residues.reserve(residues.size());
  m_residues.insert(m_residues.end(), &residues.data[0], &residues.data[residues.size()]);

  double med = getMedian(m_residues);

  m_normres.resize(residues.size());
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i]- med));
  }

  m_residues = m_normres;
  double normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  double sigma = 1.4826*normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}

template <class T>
void vpMbtTukeyEstimator<T>::psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights) {
  T cst_const = 4.6851;
  T inv_cst_const = 1 / 4.6851;
  T inv_sig = 1 / sig;

  for(size_t i = 0; i < x.size(); i++) {
#if USE_ORIGINAL_TUKEY_CODE
    if (std::fabs(sig) <= std::numeric_limits<T>::epsilon()
        && std::fabs(weights[i]) > std::numeric_limits<T>::epsilon() //sig should be equal to 0 only if NoiseThreshold == 0
        ) {
      weights[i] = 1;
      continue;
    }
#endif

    T xi_sig = x[i] * inv_sig;

    if ((std::fabs(xi_sig) <= cst_const)
#if USE_ORIGINAL_TUKEY_CODE
        && std::fabs(weights[i]) > std::numeric_limits<T>::epsilon() //Consider the previous weights here
#endif
        ) {
      //vpMath::sqr( 1 - vpMath::sqr(xi_sig * inv_cst_const) );
      weights[i] = ( 1 - (xi_sig * inv_cst_const)*(xi_sig * inv_cst_const) ) * ( 1 - (xi_sig * inv_cst_const)*(xi_sig * inv_cst_const) );
    } else {
      //Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}
