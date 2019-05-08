# Add an executable with tests, and specify the source files to compile
add_executable(test-control test-TiltCorrection.cpp)
# Link the test executable with the Google Test main entry point and the library
# under test
target_link_libraries(test-control gtest_main control)

# Add the tests to Google Test
include(GoogleTest)
gtest_discover_tests(test-control)