add_executable(parser_model_test parser_model_test.cc)
target_link_libraries(parser_model_test
  drakeLCMSystem2
  ${GTEST_BOTH_LIBRARIES})
add_test(NAME parser_model_test COMMAND parser_model_test)

