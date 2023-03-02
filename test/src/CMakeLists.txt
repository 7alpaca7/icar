cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)

# 设置输出目录结构
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# 确保输出目录存在
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
file(MAKE_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/config
    ${PROJECT_SOURCE_DIR}/src
)

# 查找依赖
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})

# 主可执行文件 - 修改了目标名称
set(MAIN_EXECUTABLE_NAME "main")  # 改为非保留名称
set(MAIN_SOURCES ${PROJECT_SOURCE_DIR}/src/test.cpp)  # 假设你的主程序是test.cpp


add_executable(${MAIN_EXECUTABLE_NAME} ${MAIN_SOURCES})

# 链接库
target_link_libraries(${MAIN_EXECUTABLE_NAME}
    ${PPNC_LIBRARIES}
    ${ONNX_LIBRARIES}
    ${OpenCV_LIBS}
    ${SERIAL_LIBRARIES}
    ${GLIB_LIBRARIES}
    pthread
)

# 复制配置文件
configure_file(
    ${PROJECT_SOURCE_DIR}/config/config.json
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config.json
    COPYONLY
)