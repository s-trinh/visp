#include <iostream>
#include <fstream>
#include <limits>
#include <stdint.h>

#include <visp3/core/vpMath.h>

#if defined (__GLIBC__)
# include <endian.h>
# if (__BYTE_ORDER == __LITTLE_ENDIAN)
#  define BOOST_LITTLE_ENDIAN
# elif (__BYTE_ORDER == __BIG_ENDIAN)
#  define BOOST_BIG_ENDIAN
# elif (__BYTE_ORDER == __PDP_ENDIAN)
#  define BOOST_PDP_ENDIAN
# else
#  error Unknown machine endianness detected.
# endif
# define BOOST_BYTE_ORDER __BYTE_ORDER
#elif defined(_BIG_ENDIAN)
# define BOOST_BIG_ENDIAN
# define BOOST_BYTE_ORDER 4321
#elif defined(_LITTLE_ENDIAN)
# define BOOST_LITTLE_ENDIAN
# define BOOST_BYTE_ORDER 1234
#elif defined(__sparc) || defined(__sparc__) \
   || defined(_POWER) || defined(__powerpc__) \
   || defined(__ppc__) || defined(__hpux) \
   || defined(_MIPSEB) || defined(_POWER) \
   || defined(__s390__)
# define BOOST_BIG_ENDIAN
# define BOOST_BYTE_ORDER 4321
#elif defined(__i386__) || defined(__alpha__) \
   || defined(__ia64) || defined(__ia64__) \
   || defined(_M_IX86) || defined(_M_IA64) \
   || defined(_M_ALPHA) || defined(__amd64) \
   || defined(__amd64__) || defined(_M_AMD64) \
   || defined(__x86_64) || defined(__x86_64__) \
   || defined(_M_X64)

# define BOOST_LITTLE_ENDIAN
# define BOOST_BYTE_ORDER 1234
#else
# error The file boost/detail/endian.hpp needs to be set up for your CPU type.
#endif

//bool isBigEndian() {
//  union {
//    uint32_t i;
//    char c[4];
//  } bint = { 0x01020304 };
//
//  return bint.c[0] == 1;
//}

uint16_t reverse16bits(const uint16_t val) {
  return ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) );
}

uint32_t reverse32bits(const uint32_t val) {
  return ( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) |
      (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) );
}

float reverseFloat(const float f) {
  union {
    float f;
    unsigned char b[4];
  } dat1, dat2;

  dat1.f = f;
  dat2.b[0] = dat1.b[3];
  dat2.b[1] = dat1.b[2];
  dat2.b[2] = dat1.b[1];
  dat2.b[3] = dat1.b[0];
  return dat2.f;
}

double reverseDouble(const double d) {
  union {
    double d;
    unsigned char b[8];
  } dat1, dat2;

  dat1.d = d;
  dat2.b[0] = dat1.b[7];
  dat2.b[1] = dat1.b[6];
  dat2.b[2] = dat1.b[5];
  dat2.b[3] = dat1.b[4];
  dat2.b[4] = dat1.b[3];
  dat2.b[5] = dat1.b[2];
  dat2.b[6] = dat1.b[1];
  dat2.b[7] = dat1.b[0];
  return dat2.d;
}

void readBinaryUShortLE(std::ifstream &file, unsigned short &ushort_value) {
  //Read
  file.read((char *)(&ushort_value), sizeof(ushort_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  ushort_value = reverse16bits(ushort_value);
#endif
}

void readBinaryShortLE(std::ifstream &file, short &short_value) {
  //Read
  file.read((char *)(&short_value), sizeof(short_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  short_value = (short) reverse16bits((uint16_t) short_value);
#endif
}

void readBinaryUIntLE(std::ifstream &file, unsigned int &uint_value) {
  //Read
  file.read((char *)(&uint_value), sizeof(uint_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  if(sizeof(uint_value) == 4) {
    uint_value = reverse32bits(uint_value);
  } else {
    uint_value = reverse16bits(uint_value);
  }
#endif
}

void readBinaryIntLE(std::ifstream &file, int &int_value) {
  //Read
  file.read((char *)(&int_value), sizeof(int_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  if(sizeof(int_value) == 4) {
    int_value = (int) reverse32bits((uint32_t) int_value);
  } else {
    int_value = reverse16bits((uint16_t) int_value);
  }
#endif
}

void readBinaryFloatLE(std::ifstream &file, float &float_value) {
  //Read
  file.read((char *)(&float_value), sizeof(float_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  float_value = reverseFloat(float_value);
#endif
}

void readBinaryDoubleLE(std::ifstream &file, double &double_value) {
  //Read
  file.read((char *)(&double_value), sizeof(double_value));

#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order from little endian to big endian
  double_value = reverseDouble(double_value);
#endif
}

void writeBinaryUShortLE(std::ofstream &file, const unsigned short ushort_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  uint16_t reverse_ushort = reverse16bits(ushort_value);
  file.write((char *)(&reverse_ushort), sizeof(reverse_ushort));
#else
  file.write((char *)(&ushort_value), sizeof(ushort_value));
#endif
}

void writeBinaryShortLE(std::ofstream &file, const short short_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  uint16_t reverse_short = reverse16bits((uint16_t) short_value);
  file.write((char *)(&reverse_short), sizeof(reverse_short));
#else
  file.write((char *)(&short_value), sizeof(short_value));
#endif
}

void writeBinaryUIntLE(std::ofstream &file, const unsigned int uint_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  if(sizeof(uint_value) == 4) {
    uint32_t reverse_uint = reverse32bits(uint_value);
    file.write((char *)(&reverse_uint), sizeof(reverse_uint));
  } else {
    uint16_t reverse_uint = reverse16bits(uint_value);
    file.write((char *)(&reverse_uint), sizeof(reverse_uint));
  }
#else
  file.write((char *)(&uint_value), sizeof(uint_value));
#endif
}

void writeBinaryIntLE(std::ofstream &file, const int int_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  if(sizeof(int_value) == 4) {
    uint32_t reverse_int = reverse32bits((uint32_t) int_value);
    file.write((char *)(&reverse_int), sizeof(reverse_int));
  } else {
    uint16_t reverse_int = reverse16bits((uint16_t) int_value);
    file.write((char *)(&reverse_int), sizeof(reverse_int));
  }
#else
  file.write((char *)(&int_value), sizeof(int_value));
#endif
}

void writeBinaryFloatLE(std::ofstream &file, const float float_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  float reverse_float = reverseFloat(float_value);
  file.write((char *)(&reverse_float), sizeof(reverse_float));
#else
  file.write((char *)(&float_value), sizeof(float_value));
#endif
}

void writeBinaryDoubleLE(std::ofstream &file, const double double_value) {
#ifdef BOOST_BIG_ENDIAN
  //Reverse bytes order to little endian
  double reverse_double = reverseDouble(double_value);
  file.write((char *)(&reverse_double), sizeof(reverse_double));
#else
  file.write((char *)(&double_value), sizeof(double_value));
#endif
}


int main(int argc, char **argv) {
  short int short_value = -30000;
  unsigned short int ushort_value = 65535;
  int int_value = -2000000000;
  unsigned int uint_value = 2147483647;
  float float_value = -1.04 * 10e-38; //-1.1754943 * 10e-38;
  double double_value = 1.5 * 10e300;

  if(argc == 1) {
    std::cout << "Write file !" << std::endl;
    std::ofstream file("test_write_binary_data.bin");
    if(file.is_open()) {
      writeBinaryShortLE(file, short_value);

      writeBinaryShortLE(file, (short) ushort_value);

      writeBinaryIntLE(file, int_value);

      writeBinaryIntLE(file, (int) uint_value);

      writeBinaryFloatLE(file, float_value);

      writeBinaryDoubleLE(file, double_value);

      file.close();
    } else {
      std::cerr << "Problem with file !" << std::endl;
    }
  }

  std::ifstream file_read("test_write_binary_data.bin");
  if(file_read.is_open()) {
    short int short_read;
    readBinaryShortLE(file_read, short_read);
    if(short_read != short_value) {
      std::cerr << "short_read=" << short_read << " ; short_value=" << short_value << std::endl;
    }

    unsigned short int ushort_read;
    readBinaryUShortLE(file_read, ushort_read);
    if(ushort_read != ushort_value) {
      std::cerr << "ushort_read=" << ushort_read << " ; ushort_value=" << ushort_value << std::endl;
    }

    int int_read;
    readBinaryIntLE(file_read, int_read);
    if(int_read != int_value) {
      std::cerr << "int_read=" << int_read << " ; int_value=" << int_value << std::endl;
    }

    unsigned int uint_read;
    readBinaryUIntLE(file_read, uint_read);
    if(uint_read != uint_value) {
      std::cerr << "uint_read=" << uint_read << " ; uint_value=" << uint_value << std::endl;
    }

    float float_read;
    readBinaryFloatLE(file_read, float_read);
    if(!vpMath::equal(float_read, float_value, std::numeric_limits<float>::epsilon())) {
      std::cerr << "float_read=" << float_read << " ; float_value=" << float_value << std::endl;
    }

    double double_read;
    readBinaryDoubleLE(file_read, double_read);
    if(!vpMath::equal(double_read, double_value, std::numeric_limits<double>::epsilon())) {
      std::cerr << "double_read=" << double_read << " ; double_value=" << double_value << std::endl;
    }
  } else {
    std::cerr << "Failed to read file=test_write_binary_data.bin !" << std::endl;
  }

  return 0;
}
