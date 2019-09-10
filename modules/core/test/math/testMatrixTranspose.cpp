#include <visp3/core/vpConfig.h>

#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include <visp3/core/vpMatrix.h>

namespace {
vpMatrix generateMatrix(unsigned int sz1, unsigned int sz2) {
  vpMatrix M(sz1, sz2);

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M[i][j] = i * M.getCols() + j;
    }
  }

  return M;
}

vpMatrix generateMatrixTranspose(unsigned int sz1, unsigned int sz2) {
  vpMatrix M(sz2, sz1);

  for (unsigned int j = 0; j < M.getCols(); j++) {
    for (unsigned int i = 0; i < M.getRows(); i++) {
      M[i][j] = j * M.getRows() + i;
    }
  }

  return M;
}

void transposeTile(const vpMatrix& a, unsigned int i, unsigned int j,
                   vpMatrix& b, unsigned int diff1, unsigned int diff2)
{
  for (unsigned int r = i; r < i + diff1; r++) {
    for (unsigned int c = j; c < j + diff2; c++) {
      b[c][r] = a[r][c];
    }
  }
}

void transposeRecursiveTileH(const vpMatrix& a,
                             unsigned int i0, unsigned int i1,
                             unsigned int j0, unsigned int j1,
                             vpMatrix& b,
                             unsigned int minTileSize)
{
  unsigned int diff = j1 - j0;
  unsigned int middle = (j0 + j1) / 2;
  if (diff > minTileSize) {
    transposeRecursiveTileH(a,
                            i0, i1,
                            j0, middle,
                            b,
                            minTileSize);
    transposeRecursiveTileH(a,
                            i0, i1,
                            middle, j1,
                            b,
                            minTileSize);
  }
  else {
    transposeTile(a, i0, j0, b, i1-i0, j1-j0);
  }
}

void transposeRecursiveTileV(const vpMatrix& a,
                             unsigned int i0, unsigned int i1,
                             unsigned int j0, unsigned int j1,
                             vpMatrix& b,
                             unsigned int minTileSize)
{
  unsigned int diff = i1 - i0;
  unsigned int middle = (i0 + i1) / 2;
  if (diff > minTileSize) {
    transposeRecursiveTileV(a,
                            i0, middle,
                            j0, j1,
                            b,
                            minTileSize);
    transposeRecursiveTileV(a,
                            middle, i1,
                            j0, j1,
                            b,
                            minTileSize);
  }
  else {
    transposeRecursiveTileH(a,
                            i0, i1,
                            j0, j1,
                            b,
                            minTileSize);
  }
}

void transposeTileSwap(const vpMatrix& a, unsigned int i, unsigned int j,
                       vpMatrix& b, unsigned int diff1, unsigned int diff2)
{
  for (unsigned int r = i; r < i + diff1; r++) {
    for (unsigned int c = j; c < j + diff2; c++) {
      b[c][r] = a[r][c];
      b[r][c] = a[c][r];
    }
  }
}

void transposeRecursiveTileSquareSwap(const vpMatrix& a,
                                      unsigned int i0, unsigned int i1,
                                      unsigned int j0, unsigned int j1,
                                      vpMatrix& b,
                                      unsigned int minTileSize)
{
  unsigned int diff = i1 - i0;
  unsigned int middle = (i0 + i1) / 2;
  if (diff > minTileSize) {
#if 0
        // upper left
    transposeRecursiveTileSquareSwap(a,
                                     i0, middle,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // upper right
    transposeRecursiveTileSquareSwap(a,
                                     i0, middle,
                                     middle, j1,
                                     b,
                                     minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(a,
                                     middle, i1,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquareSwap(a,
                                     middle, i1,
                                     middle, j1,
                                     b,
                                     minTileSize);
#else
        // upper left
    transposeRecursiveTileSquareSwap(a,
                                     i0, middle,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(a,
                                     middle, i1,
                                     j0, middle,
                                     b,
                                     minTileSize);
    // upper right
    transposeRecursiveTileSquareSwap(a,
                                     i0, middle,
                                     middle, j1,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquareSwap(a,
                                     middle, i1,
                                     middle, j1,
                                     b,
                                     minTileSize);
#endif
  }
  else {
    transposeTileSwap(a,
                      i0, j0,
                      b,
                      i1-i0,
                      j1-j0);
  }
}

void transposeRecursiveTileSquare(const vpMatrix& a,
                                  unsigned int start, unsigned int end,
                                  vpMatrix& b,
                                  unsigned int minTileSize)
{
  unsigned int diff = end - start;
  unsigned int middle = (start + end) / 2;
  if (diff > minTileSize) {
#if 0
        // upper left
    transposeRecursiveTileSquare(a,
                                 start,
                                 middle,
                                 b,
                                 minTileSize);
    // lower right
    transposeRecursiveTileSquare(a,
                                 middle,
                                 end,
                                 b,
                                 minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(a,
                                     middle, end,
                                     start, middle,
                                     b,
                                     minTileSize);
#else
        // upper left
    transposeRecursiveTileSquare(a,
                                 start,
                                 middle,
                                 b,
                                 minTileSize);
    // lower left
    transposeRecursiveTileSquareSwap(a,
                                     middle, end,
                                     start, middle,
                                     b,
                                     minTileSize);
    // lower right
    transposeRecursiveTileSquare(a,
                                 middle,
                                 end,
                                 b,
                                 minTileSize);
#endif
  }
  else {
    transposeTile(a, start, start, b, end - start, end - start);
  }
}

vpMatrix transposeRecursive(const vpMatrix& a, unsigned int minTileSize=8, bool squareProc=true)
{
  vpMatrix b;
  b.resize(a.getCols(), a.getRows(), false, false);

  const unsigned int size1 = a.getRows();
  const unsigned int size2 = a.getCols();

  if (a.getRows() == a.getCols() && squareProc) {
    transposeRecursiveTileSquare(a,
                                 0, size1,
                                 b,
                                 minTileSize);
  }
  else {
    transposeRecursiveTileV(a,
                            0, size1,
                            0, size2,
                            b,
                            minTileSize);
  }

  return b;
}

vpMatrix transposeTiling(const vpMatrix& a, int tileSize = 32)
{
  vpMatrix b;
  b.resize(a.getCols(), a.getRows(), false, false);

  const int nrows = static_cast<int>(a.getRows());
  const int ncols = static_cast<int>(a.getCols());
  for (int i = 0; i < nrows;) {
    for (; i <= nrows - tileSize; i += tileSize) {
      for (int k = i; k < i + tileSize; k++) {
        for (int j = 0; j < ncols;) {
          for (; j <= ncols - tileSize; j += tileSize) {
            for (int l = j; l < j + tileSize; l++) {
              b[l][k] = a[k][l];
            }
          }

          for (; j < ncols; j++) {
            b[j][k] = a[k][j];
          }
        }
      }
    }

    for (; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        b[j][i] = a[i][j];
      }
    }
  }

  return b;
}

TEST_CASE("Benchmark vpMatrix transpose", "[benchmark]") {
  const std::vector<std::pair<int, int>> sizes = { {65, 65}, {137, 137}, {1201, 1201}, {1024, 1024},
                                                   {64, 128}, {128, 64}, {512, 1024}, {1024, 512}, {64, 1024}, {1024, 64},
                                                   {6, 1024}, {1024, 6}, {6, 2048}, {2048, 6},
                                                   {701, 1503}, {1791, 837}/*,
                                                   {10000, 10000}*/ };

  //{
  //  const int nrows = 2, ncols = 2, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}
  //{
  //  const int nrows = 1, ncols = 2, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}
  //{
  //  const int nrows = 2, ncols = 1, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}
  //{
  //  const int nrows = 1, ncols = 1, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}

  //{
  //  const int nrows = 3, ncols = 6, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}
  //{
  //  const int nrows = 6, ncols = 3, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}

  //{
  //  const int nrows = 3, ncols = 7, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}
  //{
  //  const int nrows = 7, ncols = 3, tileSize = 2;
  //  vpMatrix M = generateMatrix(nrows, ncols);
  //  vpMatrix Mt_true = generateMatrixTranspose(nrows, ncols);
  //  vpMatrix Mt = transposeTiling(M, tileSize);
  //  std::cout << "\n(Mt == Mt_true): " << (Mt == Mt_true) << std::endl;
  //  std::cout << "Mt_true:\n" << Mt_true << std::endl;
  //  std::cout << "Mt:\n" << Mt << std::endl;
  //}

  for (auto sz : sizes) {
    vpMatrix M = generateMatrix(sz.first, sz.second);
    vpMatrix Mt_true = generateMatrixTranspose(sz.first, sz.second);

    std::ostringstream oss;
    oss << "Benchmark vpMatrix M.t(), size=";
    oss << sz.first << "x" << sz.second;
    BENCHMARK(oss.str().c_str()) {
      vpMatrix Mt = M.t();
      REQUIRE(Mt == Mt_true);
      return Mt;
    };

    oss.str("");
    oss << "Benchmark vpMatrix M.transpose(), size=";
    oss << sz.first << "x" << sz.second;
    BENCHMARK(oss.str().c_str()) {
      vpMatrix Mt = M.transpose();
      REQUIRE(Mt == Mt_true);
      return Mt;
    };

    oss.str("");
    oss << "Benchmark vpMatrix M.transpose(Mt), size=";
    oss << sz.first << "x" << sz.second;
    BENCHMARK(oss.str().c_str()) {
      vpMatrix Mt;
      M.transpose(Mt);
      REQUIRE(Mt == Mt_true);
      return Mt;
    };

    oss.str("");
    oss << "Benchmark transpose tiling 8x8, size=";
    oss << sz.first << "x" << sz.second;
    BENCHMARK(oss.str().c_str()) {
      vpMatrix Mt = transposeTiling(M, 8);
      REQUIRE(Mt == Mt_true);
      return Mt;
    };

    if (sz.first == sz.second) {
      oss.str("");
      oss << "Benchmark recursive transpose 8x8 square proc, size=";
      oss << sz.first << "x" << sz.second;
      BENCHMARK(oss.str().c_str()) {
        vpMatrix Mt = transposeRecursive(M, 8, true);
        REQUIRE(Mt == Mt_true);
        return Mt;
      };
    }

    oss.str("");
    oss << "Benchmark recursive transpose 8x8, size=";
    oss << sz.first << "x" << sz.second;
    BENCHMARK(oss.str().c_str()) {
      vpMatrix Mt = transposeRecursive(M, 8, false);
      REQUIRE(Mt == Mt_true);
      return Mt;
    };

    //oss.str("");
    //oss << "Benchmark recursive transpose 2x2, size=";
    //oss << sz.first << "x" << sz.second;
    //BENCHMARK(oss.str().c_str()) {
    //  vpMatrix Mt = transposeRecursive(M, 2);
    //  REQUIRE(Mt == Mt_true);
    //  return Mt;
    //};
  }
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
