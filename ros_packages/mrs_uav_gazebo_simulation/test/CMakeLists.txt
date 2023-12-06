find_package(rostest REQUIRED)

### WARNING
### NEVER name your test build unit the same as the folder that it is placed in!
### e.g., this will cause problems:
###   add_rostest_gtest(my_test
###     my_test/basic.test
###     my_test/test.cpp
###     )

# basic test

add_rostest_gtest(test_basic
  basic/basic.test
  basic/test.cpp
  )

target_link_libraries(test_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(test_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
