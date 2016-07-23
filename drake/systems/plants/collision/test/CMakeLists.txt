if(bullet_FOUND)
  add_executable(collision_model_test collision_model_test.cc)
  target_link_libraries(collision_model_test drakeCollision ${GTEST_BOTH_LIBRARIES})
  add_test(NAME collision_model_test COMMAND collision_model_test)
endif()

add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions COMMAND testActiveCollisionOptions)
