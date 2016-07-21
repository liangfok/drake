# TODO(#2852) We can't yet handle shadowing as a compiler error.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS_NO_ERROR_SHADOW}")

add_library_with_exports(LIB_NAME drakeParsers SOURCE_FILES
  model.cc)
add_dependencies(drakeParsers drakeRBM)
target_link_libraries(drakeParsers
  drakeRBM)

pods_install_libraries(drakeParsers)
drake_install_headers(
  model.h)
pods_install_pkg_config_file(drake-parsers
  LIBS -ldrakeParsers
  REQUIRES lcm
  VERSION 0.0.1)

add_subdirectory(test)
