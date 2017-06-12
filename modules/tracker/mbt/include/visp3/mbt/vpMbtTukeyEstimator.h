#ifndef __vpMbtTukeyEstimator_h_
#define __vpMbtTukeyEstimator_h_

#include <vector>
#include <visp3/core/vpColVector.h>

template <typename T>
class VISP_EXPORT vpMbtTukeyEstimator {
public:
  T getMedian(std::vector<T> &vec);

  void MEstimator(const std::vector<T> &residues, std::vector<T> &weights, const T NoiseThreshold);
  void MEstimator(const vpColVector &residues, vpColVector &weights, const double NoiseThreshold);

  void psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights);
  void psiTukey(const double sig, std::vector<double> &x, vpColVector &weights);

private:
  std::vector<T> m_normres;
  std::vector<T> m_residues;
};
#endif
