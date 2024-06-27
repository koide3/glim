find_path(GTSAM_INCLUDE_DIRS gtsam/inference/FactorGraph.h
  HINTS /usr/local/include /usr/include
  DOC "GTSAM include directories")

find_library(GTSAM_LIB NAMES gtsam
  HINTS /usr/local/lib /usr/lib
  DOC "GTSAM libraries")

find_library(GTSAM_UNSTABLE_LIB NAMES gtsam_unstable
  HINTS /usr/local/lib /usr/lib
  DOC "GTSAM_UNSTABLE libraries")

find_library(TBB_LIB NAMES tbb
  HINTS /usr/local/lib /usr/lib
  DOC "TBB libraries")

find_library(TBB_MALLOC_LIB NAMES tbbmalloc
  HINTS /usr/local/lib /usr/lib
  DOC "TBB malloc libraries")

if(GTSAM_LIB AND GTSAM_UNSTABLE_LIB AND TBB_LIB)
  set(GTSAM_LIBRARIES ${GTSAM_LIB} ${GTSAM_UNSTABLE_LIB} ${TBB_LIB} ${TBB_MALLOC_LIB})
endif()

if(NOT TARGET GTSAM::GTSAM)
  add_library(GTSAM::GTSAM INTERFACE IMPORTED GLOBAL)
  set_target_properties(GTSAM::GTSAM PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${GTSAM_LIBRARIES}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM DEFAULT_MSG GTSAM_INCLUDE_DIRS GTSAM_LIBRARIES)
