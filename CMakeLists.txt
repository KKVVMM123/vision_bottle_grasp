cmake_minimum_required(VERSION 3.10)
project(bottle_grasp)

set(CMAKE_CXX_STANDARD 17)

# 启用 pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/eigen3
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include
)

set(SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/Ti5_socketcan.cpp
    ${PROJECT_SOURCE_DIR}/src/aoyi_hand.cpp
    ${PROJECT_SOURCE_DIR}/src/aoyihand.cpp
    ${PROJECT_SOURCE_DIR}/src/function.cpp
    ${PROJECT_SOURCE_DIR}/src/kinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/motion.cpp
    ${PROJECT_SOURCE_DIR}/src/vision_client.cpp
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)

add_executable(bottle_grasp ${SRC_FILES})

link_directories(${PROJECT_SOURCE_DIR}/lib /usr/local/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# serial 库（商超工程同样依赖）
find_library(SERIAL_LIB serial
    PATHS
    /usr/local/lib
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
if(NOT SERIAL_LIB)
    find_library(SERIAL_LIB libserial.a
        PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/aarch64-linux-gnu
    )
endif()
if(NOT SERIAL_LIB)
    message(FATAL_ERROR "serial library not found. Please install libserial.")
endif()

target_link_libraries(bottle_grasp
    ${SERIAL_LIB}
    Threads::Threads
    ${LIBUSB_LIBRARIES}
    ${CMAKE_DL_LIBS}
)
