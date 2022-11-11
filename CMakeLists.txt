cmake_minimum_required(VERSION 3.0.0)
project(
    sf45
    VERSION 0.1.0
    DESCRIPTION "Serial interface for Lightware SF45")

include(CTest)
enable_testing()

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
)

add_executable(sf45 main.cpp ${all_SRCS} lwNx.cpp "win32/lwSerialPortWin32.cpp" "win32/platformWin32.cpp")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

target_compile_features(sf45 PUBLIC cxx_std_20)