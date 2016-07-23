# TODO(#2852) We can't yet handle shadowing as a compiler error.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS_NO_ERROR_SHADOW}")

add_executable(model_test model_test.cc)
target_link_libraries(model_test
  drakeParsers
  ${GTEST_BOTH_LIBRARIES})
add_test(NAME model_test COMMAND model_test)

