#include <visp3/core/vpConfig.h>

#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#define DEBUG 0

namespace {
bool equal(double x, double y, double tol)
{
  return std::fabs(x - y) < tol;
}

class MatrixXd {
public:
  MatrixXd() :
    m_data(), m_rows(0), m_cols(0) {}

  MatrixXd(int row, int col) :
    m_data(row*col), m_rows(row), m_cols(col) {}

  double& operator() (int row, int col)
  {
    return m_data[row*m_cols + col];
  }

  double operator() (int row, int col) const
  {
    return m_data[row*m_cols + col];
  }

  MatrixXd t1() const
  {
    MatrixXd At(m_cols, m_rows);

    for (int i = 0; i < m_rows; i++) {
      for (int j = 0; j < m_cols; j++) {
        At(j, i) = (*this)(i, j);
      }
    }
    return At;
  }

  MatrixXd t2() const
  {
    MatrixXd At(m_cols, m_rows);

    for (int i = 0; i < m_cols; i++) {
      for (int j = 0; j < m_rows; j++) {
        At(i, j) = (*this)(j, i);
      }
    }
    return At;
  }

  MatrixXd transposeTiling(int tileSize = 32) const;

  MatrixXd transposeRecursive(int minTileSize = 8, bool squareProc = true) const;

  void transposeRecursiveTileSquare(int start, int end, MatrixXd& b, int minTileSize) const;

  void transposeRecursiveTileSquareSwap(int i0, int i1, int j0, int j1, MatrixXd& b, int minTileSize) const;

  void transposeTileSwap(int i, int j, MatrixXd& b, int diff1, int diff2) const;

  void transposeRecursiveTileV(int i0, int i1, int j0, int j1, MatrixXd& b, int minTileSize) const;

  void transposeRecursiveTileH(int i0, int i1, int j0, int j1, MatrixXd& b, int minTileSize) const;

  void transposeTile(int i, int j, MatrixXd& b, int diff1, int diff2) const;

  bool operator==(const MatrixXd& b) const
  {
    if (b.m_rows != m_rows || b.m_cols != m_cols) {
      return false;
    }

    for (int i = 0; i < m_rows; i++) {
      for (int j = 0; j < m_cols; j++) {
        if (!equal((*this)(i, j), b(i, j), std::numeric_limits<double>::epsilon())) {
          return false;
        }
      }
    }

    return true;
  }

  friend std::ostream &operator<<(std::ostream &s, const MatrixXd& A)
  {
    std::ios_base::fmtflags original_flags = s.flags();

    s.precision(10);
    for (int i = 0; i < A.m_rows; i++) {
      for (int j = 0; j < A.m_cols - 1; j++) {
        s << A(i, j) << "  ";
      }
      // We don't add "  " after the last row element
      s << A(i, A.m_cols - 1);
      // We don't add a \n char on the end of the last array line
      if (i < A.m_rows - 1) {
        s << std::endl;
      }
    }

    s.flags(original_flags); // restore s to standard state

    return s;
  }

  std::vector<double> m_data;
  int m_rows;
  int m_cols;
};

MatrixXd generateMatrix(int sz1, int sz2) {
  MatrixXd M(sz1, sz2);

  for (int i = 0; i < M.m_rows; i++) {
    for (int j = 0; j < M.m_cols; j++) {
      M(i, j) = i * M.m_cols + j;
    }
  }

  return M;
}

MatrixXd generateMatrixTranspose(int sz1, int sz2) {
  MatrixXd M(sz2, sz1);

  for (int j = 0; j < M.m_cols; j++) {
    for (int i = 0; i < M.m_rows; i++) {
      M(i, j) = j * M.m_rows + i;
    }
  }

  return M;
}

void MatrixXd::transposeTile(int i, int j,
                             MatrixXd& b, int diff1, int diff2) const
{
  for (int r = i; r < i + diff1; r++) {
    for (int c = j; c < j + diff2; c++) {
#if DEBUG
      std::cout << "r: " << r << " ; c: " << c << std::endl;
#endif
      b(c, r) = (*this)(r, c);
    }
  }
}

void MatrixXd::transposeRecursiveTileH(int i0, int i1,
                                       int j0, int j1,
                                       MatrixXd& b,
                                       int minTileSize) const
{
  int diff = j1 - j0;
  int middle = (j0 + j1) / 2;
  if (diff > minTileSize) {
    transposeRecursiveTileH(i0, i1,
                            j0, middle,
                            b,
                            minTileSize);
    transposeRecursiveTileH(i0, i1,
                            middle, j1,
                            b,
                            minTileSize);
  }
  else {
    transposeTile(i0, j0, b, i1-i0, j1-j0);
  }
}

void MatrixXd::transposeRecursiveTileV(int i0, int i1,
                                       int j0, int j1,
                                       MatrixXd& b,
                                       int minTileSize) const
{
  int diff = i1 - i0;
  int middle = (i0 + i1) / 2;
  if (diff > minTileSize) {
    transposeRecursiveTileV(i0, middle,
                            j0, j1,
                            b,
                            minTileSize);
    transposeRecursiveTileV(middle, i1,
                            j0, j1,
                            b,
                            minTileSize);
  }
  else {
    transposeRecursiveTileH(i0, i1,
                            j0, j1,
                            b,
                            minTileSize);
  }
}

void MatrixXd::transposeTileSwap(int i, int j,
                                 MatrixXd& b, int diff1, int diff2) const
{
  for (int r = i; r < i + diff1; r++) {
    for (int c = j; c < j + diff2; c++) {
      b(c, r) = (*this)(r, c);
      b(r, c) = (*this)(c, r);
    }
  }
}

void MatrixXd::transposeRecursiveTileSquareSwap(int i0, int i1,
                                                int j0, int j1,
                                                MatrixXd& b,
                                                int minTileSize) const
{
  int diff = i1 - i0;
  int middle = (i0 + i1) / 2;
  if (diff > minTileSize) {
#if 0
        // upper left
    transposeRecursiveTileSquareSwap(i0, middle,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // upper right
    transposeRecursiveTileSquareSwap(i0, middle,
                                     middle, j1,
                                     b,
                                     minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(middle, i1,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquareSwap(middle, i1,
                                     middle, j1,
                                     b,
                                     minTileSize);
#else
        // upper left
    transposeRecursiveTileSquareSwap(i0, middle,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(middle, i1,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // upper right
    transposeRecursiveTileSquareSwap(i0, middle,
                                     middle, j1,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquareSwap(middle, i1,
                                     middle, j1,
                                     b,
                                     minTileSize);
#endif
  }
  else {
    transposeTileSwap(i0, j0,
                      b,
                      i1-i0,
                      j1-j0);
  }
}

void MatrixXd::transposeRecursiveTileSquare(int start, int end,
                                            MatrixXd& b,
                                            int minTileSize) const
{
  int diff = end - start;
  int middle = (start + end) / 2;
  if (diff > minTileSize) {
#if 0
        // upper left
    transposeRecursiveTileSquare(start,
                                 middle,
                                 b,
                                 minTileSize);
    // lower right
    transposeRecursiveTileSquare(middle,
                                 end,
                                 b,
                                 minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(middle, end,
                                     start, middle,
                                     b,
                                     minTileSize);
#else
        // upper left
    transposeRecursiveTileSquare(start,
                                 middle,
                                 b,
                                 minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(middle, end,
                                     start, middle,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquare(middle,
                                 end,
                                 b,
                                 minTileSize);
#endif
  }
  else {
    transposeTile(start, start, b, end - start, end - start);
  }
}

MatrixXd MatrixXd::transposeRecursive(int minTileSize, bool squareProc) const
{
  MatrixXd b(m_cols, m_rows);

  const int size1 = m_rows;
  const int size2 = m_cols;

  if (m_rows == m_cols && squareProc) {
    transposeRecursiveTileSquare(0, size1,
                                 b,
                                 minTileSize);
  }
  else {
    transposeRecursiveTileV(0, size1,
                            0, size2,
                            b,
                            minTileSize);
  }

  return b;
}

MatrixXd MatrixXd::transposeTiling(int tileSize) const
{
  MatrixXd b(m_cols, m_rows);

  const int nrows = static_cast<int>(m_rows);
  const int ncols = static_cast<int>(m_cols);

#if 0
  for (int i = 0; i < nrows;) {
    for (; i <= nrows - tileSize; i += tileSize) {
      for (int k = i; k < i + tileSize; k++) {
        for (int j = 0; j < ncols;) {
          for (; j <= ncols - tileSize; j += tileSize) {
            for (int l = j; l < j + tileSize; l++) {
#if DEBUG
              std::cout << "k: " << k << " ; l: " << l << std::endl;
#endif
              b(l, k) = (*this)(k, l);
            }
          }

          for (; j < ncols; j++) {
            b(j, k) = (*this)(k, j);
          }
        }
      }
    }

    for (; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        b(j, i) = (*this)(i, j);
      }
    }
  }
#else
  for (int i = 0; i < nrows;) {
    for (; i <= nrows - tileSize; i += tileSize) {
      int j = 0;
      for (; j <= ncols - tileSize; j += tileSize) {
        for (int k = i; k < i + tileSize; k++) {
          for (int l = j; l < j + tileSize; l++) {
#if 1 //DEBUG
            std::cout << "k: " << k << " ; l: " << l << std::endl;
#endif
            b(l, k) = (*this)(k, l);
          }
        }
      }

      for (int k = i; k < i + tileSize; k++) {
        for (int l = j; l < ncols; l++) {
          b(l, k) = (*this)(k, l);
        }
      }
    }

    for (; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        b(j, i) = (*this)(i, j);
      }
    }
  }
#endif

  return b;
}

TEST_CASE("Benchmark vpMatrix transpose", "[benchmark]") {
  const std::vector<std::pair<int, int>> sizes = { {65, 65}, {137, 137}, {1201, 1201}, {1024, 1024},
                                                   {64, 128}, {128, 64}, {512, 1024}, {1024, 512}, {64, 1024}, {1024, 64},
                                                   {6, 1024}, {1024, 6}, {6, 2048}, {2048, 6},
                                                   {701, 1503}, {1791, 837}/*,
                                                   {10000, 10000}*/ };

  //const int nrows = 8, ncols = 8;
  //MatrixXd M = generateMatrix(nrows, ncols);
  //MatrixXd Mt_true = generateMatrixTranspose(nrows, ncols);
  //std::cout << "transposeTiling:" << std::endl;
  //MatrixXd Mt_tiling = M.transposeTiling(2);
  //std::cout << "\ntransposeRecursive:" << std::endl;
  //MatrixXd Mt_recursive = M.transposeRecursive(2, false);

  {
    const int nrows = 2, ncols = 2, tileSize = 2;
    MatrixXd M = generateMatrix(nrows, ncols);
    MatrixXd Mt_true = generateMatrixTranspose(nrows, ncols);
    MatrixXd Mt = M.transposeTiling(tileSize);
    std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
    std::cout << "Mt_true:\n" << Mt_true << std::endl;
    std::cout << "Mt:\n" << Mt << std::endl;
  }
  {
    const int nrows = 1, ncols = 2, tileSize = 2;
    MatrixXd M = generateMatrix(nrows, ncols);
    MatrixXd Mt_true = generateMatrixTranspose(nrows, ncols);
    MatrixXd Mt = M.transposeTiling(tileSize);
    std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
    std::cout << "Mt_true:\n" << Mt_true << std::endl;
    std::cout << "Mt:\n" << Mt << std::endl;
  }
  {
    const int nrows = 2, ncols = 1, tileSize = 2;
    MatrixXd M = generateMatrix(nrows, ncols);
    MatrixXd Mt_true = generateMatrixTranspose(nrows, ncols);
    MatrixXd Mt = M.transposeTiling(tileSize);
    std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
    std::cout << "Mt_true:\n" << Mt_true << std::endl;
    std::cout << "Mt:\n" << Mt << std::endl;
  }

  //for (auto sz : sizes) {
  //  MatrixXd M = generateMatrix(sz.first, sz.second);
  //  MatrixXd Mt_true = generateMatrixTranspose(sz.first, sz.second);

  //  std::ostringstream oss;
  //  oss << sz.first << "x" << sz.second;
  //  oss << " - M.t1()";
  //  BENCHMARK(oss.str().c_str()) {
  //    MatrixXd Mt = M.t1();
  //    REQUIRE(Mt == Mt_true);
  //    return Mt;
  //  };

  //  oss.str("");
  //  oss << sz.first << "x" << sz.second;
  //  oss << " - M.t2()";
  //  BENCHMARK(oss.str().c_str()) {
  //    MatrixXd Mt = M.t2();
  //    REQUIRE(Mt == Mt_true);
  //    return Mt;
  //  };

  //  oss.str("");
  //  oss << sz.first << "x" << sz.second;
  //  oss << " - transpose tiling 8x8";
  //  BENCHMARK(oss.str().c_str()) {
  //    MatrixXd Mt = M.transposeTiling(8);
  //    REQUIRE(Mt == Mt_true);
  //    return Mt;
  //  };

  //  if (sz.first == sz.second) {
  //    oss.str("");
  //    oss << sz.first << "x" << sz.second;
  //    oss << " - recursive transpose 8x8 square";
  //    BENCHMARK(oss.str().c_str()) {
  //      MatrixXd Mt = M.transposeRecursive(8, true);
  //      REQUIRE(Mt == Mt_true);
  //      return Mt;
  //    };
  //  }

  //  oss.str("");
  //  oss << sz.first << "x" << sz.second;
  //  oss << " - recursive transpose 8x8";
  //  BENCHMARK(oss.str().c_str()) {
  //    MatrixXd Mt = M.transposeRecursive(8, false);
  //    REQUIRE(Mt == Mt_true);
  //    return Mt;
  //  };
  //}
}
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
