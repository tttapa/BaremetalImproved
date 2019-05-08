# Create target and set properties
add_library(src-shared INTERFACE)

# Set target properties
target_include_directories(src-shared
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(src-shared INTERFACE cxx_std_17)

if(NOT CMAKE_CROSSCOMPILING)
add_subdirectory(test)
endif()

install(TARGETS src-shared
        EXPORT src-shared
        INCLUDES DESTINATION include)
install(EXPORT src-shared DESTINATION lib)
install(
FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication/BaremetalCommuncationDef.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication/LogEntry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication/SharedStruct.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/Array.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/ArrayHelpers.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/HouseholderQR.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/LeastSquares.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/Matrix.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matrix/MatrixHelpers.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/quaternion/EulerAngles.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/quaternion/Quaternion.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities/real_t.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities/Square.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vivado-types/OutputTypes.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vivado-types/SensorTypes.hpp
DESTINATION include)