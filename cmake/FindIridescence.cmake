find_path(Iridescence_INCLUDE_DIRS glk/drawable.hpp
  HINTS /usr/local/include/iridescence /usr/include/iridescence
  DOC "Iridescence include directories")

find_library(Iridescence_LIBRARY NAMES iridescence
  HINTS /usr/local/lib /usr/lib
  DOC "Iridescence libraries")

  find_library(gl_imgui_LIBRARY NAMES gl_imgui
  HINTS /usr/local/lib /usr/lib
  DOC "Iridescence libraries")

set(Iridescence_LIBRARIES ${Iridescence_LIBRARY} ${gl_imgui_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Iridescence DEFAULT_MSG Iridescence_INCLUDE_DIRS Iridescence_LIBRARIES)
