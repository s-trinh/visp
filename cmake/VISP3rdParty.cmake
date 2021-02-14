#----------------------------------------------------------------------
# Add 3rd-party libraries build as static libs
#----------------------------------------------------------------------

if(WITH_ATIDAQ)
  set(ATIDAQ_LIBRARY visp_atidaq)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/atidaq")
  set(ATIDAQ_INCLUDE_DIRS "${${ATIDAQ_LIBRARY}_SOURCE_DIR}" "${${ATIDAQ_LIBRARY}_BINARY_DIR}")
  set(ATIDAQ_LIBRARIES ${ATIDAQ_LIBRARY})
  set(ATIDAQ_VERSION ${ATIDAQ_MAJOR_VERSION}.${ATIDAQ_MINOR_VERSION}.${ATIDAQ_PATCH_VERSION})
endif()

if(WITH_CLIPPER)
  set(CLIPPER_LIBRARY visp_clipper)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clipper")
  set(CLIPPER_INCLUDE_DIRS "${${CLIPPER_LIBRARY}_SOURCE_DIR}" "${${CLIPPER_LIBRARY}_BINARY_DIR}")
  set(CLIPPER_LIBRARIES ${CLIPPER_LIBRARY})
  set(CLIPPER_VERSION ${CLIPPER_MAJOR_VERSION}.${CLIPPER_MINOR_VERSION}.${CLIPPER_PATCH_VERSION})
endif()

if(WITH_LAPACK)
  set(LAPACK_LIBRARY visp_lapack)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clapack")
  set(LAPACK_LIBRARIES ${LAPACK_LIBRARY})
  set(LAPACK_VERSION ${LAPACK_MAJOR_VERSION}.${LAPACK_MINOR_VERSION}.${LAPACK_PATCH_VERSION})
endif()

if(WITH_PTHREAD)
  set(PTHREADS_LIBRARY visp_pthread)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/pthreads4w")
  set(PTHREADS_INCLUDE_DIRS "${${PTHREADS_LIBRARY}_SOURCE_DIR}" "${${PTHREADS_LIBRARY}_BINARY_DIR}")
  set(PTHREADS_LIBRARIES ${PTHREADS_LIBRARY})
  set(PTHREADS_VERSION ${PTHREADS_MAJOR_VERSION}.${PTHREADS_MINOR_VERSION}.${PTHREADS_PATCH_VERSION})
endif()

if(WITH_APRILTAG)
  set(APRILTAG_LIBRARY visp_apriltag)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/apriltag")
  set(APRILTAG_INCLUDE_DIRS "${${APRILTAG_LIBRARY}_SOURCE_DIR}" "${${APRILTAG_LIBRARY}_BINARY_DIR}")
  set(APRILTAG_LIBRARIES ${APRILTAG_LIBRARY})
  set(APRILTAG_VERSION ${APRILTAG_MAJOR_VERSION}.${APRILTAG_MINOR_VERSION}.${APRILTAG_PATCH_VERSION})
endif()

if(WITH_QBDEVICE)
  set(QBDEVICE_LIBRARY visp_qbdevice)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/qbdevice")
  set(QBDEVICE_INCLUDE_DIRS "${${QBDEVICE_LIBRARY}_SOURCE_DIR}" "${${QBDEVICE_LIBRARY}_BINARY_DIR}")
  set(QBDEVICE_LIBRARIES ${QBDEVICE_LIBRARY})
  set(QBDEVICE_VERSION ${QBDEVICE_MAJOR_VERSION}.${QBDEVICE_MINOR_VERSION}.${QBDEVICE_PATCH_VERSION})
endif()

if(WITH_TAKKTILE2)
  set(TAKKTILE2_LIBRARY visp_reflex_takktile2)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/reflex-takktile2")
  set(TAKKTILE2_INCLUDE_DIRS "${${TAKKTILE2_LIBRARY}_SOURCE_DIR}" "${${TAKKTILE2_LIBRARY}_BINARY_DIR}")
  set(TAKKTILE2_LIBRARIES ${TAKKTILE2_LIBRARY})
  set(TAKKTILE2_VERSION ${TAKKTILE2_MAJOR_VERSION}.${TAKKTILE2_MINOR_VERSION}.${TAKKTILE2_PATCH_VERSION})
endif()

# pugixml is always enabled to provide default XML I/O capabilities
set(PUGIXML_LIBRARY visp_pugixml)
add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/pugixml-1.9")
set(PUGIXML_INCLUDE_DIRS "${${PUGIXML_LIBRARY}_SOURCE_DIR}" "${${PUGIXML_LIBRARY}_BINARY_DIR}")
set(PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
set(PUGIXML_VERSION ${PUGIXML_MAJOR_VERSION}.${PUGIXML_MINOR_VERSION}.${PUGIXML_PATCH_VERSION})

# simdlib is always enabled since it contains fallback code to plain C++ code
set(SIMD_LIBRARY visp_simdlib)
add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/simdlib")
set(SIMDLIB_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/simdlib")
set(SIMDLIB_LIBRARIES ${SIMD_LIBRARY})

if(WITH_STBIMAGE)
  set(STBIMAGE_LIBRARY visp_stbimage)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/stb_image")
  set(STBIMAGE_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/stb_image")
  set(STBIMAGE_VERSION ${STBIMAGE_MAJOR_VERSION}.${STBIMAGE_MINOR_VERSION}.${STBIMAGE_PATCH_VERSION})
endif()

if(WITH_CATCH2)
  set(CATCH2_LIBRARY visp_catch2)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/catch2")
  set(CATCH2_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/catch2")
  set(CATCH2_VERSION ${CATCH2_MAJOR_VERSION}.${CATCH2_MINOR_VERSION}.${CATCH2_PATCH_VERSION})
endif()

#TODO: check for C++11 availability for toojpeg
# toojpeg is always enabled
set(TOOJPEG_LIBRARY visp_toojpeg)
add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/toojpeg")
set(TOOJPEG_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/toojpeg")
set(TOOJPEG_LIBRARIES ${TOOJPEG_LIBRARY})
set(TOOJPEG_VERSION ${TOOJPEG_MAJOR_VERSION}.${TOOJPEG_MINOR_VERSION})

#TODO: check if it can always be available for better PNG compression?
# miniz is always enabled
set(MINIZ_LIBRARY visp_miniz)
add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/miniz")
set(MINIZ_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/miniz")
set(MINIZ_LIBRARIES ${MINIZ_LIBRARY})
set(MINIZ_VERSION ${MINIZ_MAJOR_VERSION}.${MINIZ_MINOR_VERSION}.${MINIZ_PATCH_VERSION})
