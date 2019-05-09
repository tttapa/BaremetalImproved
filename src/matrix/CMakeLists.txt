# Create target and set properties
add_library(matrix INTERFACE)

#Set target properties
target_include_directories(matrix
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(matrix INTERFACE cxx_std_17)

target_link_libraries(matrix INTERFACE utilities)

if(NOT CMAKE_CROSSCOMPILING)
add_subdirectory(test)
endif()

install(TARGETS matrix
        EXPORT matrix
        INCLUDES DESTINATION include)
install(EXPORT matrix DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Array.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ArrayHelpers.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/HouseholderQR.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/LeastSquares.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Matrix.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/MatrixHelpers.hpp
DESTINATION include)