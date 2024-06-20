cmake_minimum_required (VERSION 3.5)
project(ParseTool)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Bin/) #执行文件 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Bin/lib) #动态库
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/)
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/Camera)
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/Ego)
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/Gnss)
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/Lidar)
include_directories(${CMAKE_SOURCE_DIR}/ParseTool/FileTool)
include_directories(${CMAKE_SOURCE_DIR}/ReWriteTool)
include_directories(${CMAKE_SOURCE_DIR}/PreprocessTool)
include_directories(${CMAKE_SOURCE_DIR}/GeomTool)
# include_directories("/home/liulei10/.local/lib/python3.8/site-packages/numpy/core/include")

aux_source_directory(${CMAKE_SOURCE_DIR}/ParseTool/Camera/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/ParseTool/Ego/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/ParseTool/Gnss/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/ParseTool/Lidar/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/ParseTool/FileTool/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/ReWriteTool/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/PreprocessTool/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/GeomTool/ ALL_MY_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/GeomTool/LaneLine ALL_MY_SRC)
list(APPEND ALL_MY_SRC ${CMAKE_SOURCE_DIR}/GeomTool/Obstacle/ObstacleTool.cpp)
# aux_source_directory(${CMAKE_SOURCE_DIR}/GeomTool/Obstacle ALL_MY_SRC)
include_directories(/usr/include/python3.8/)
include_directories(${CMAKE_SOURCE_DIR}/pybind11/include)
include_directories(${CMAKE_SOURCE_DIR}/TPL/jsoncpp/include)
include_directories(${CMAKE_SOURCE_DIR}/TPL/toml11/include)
include_directories(${CMAKE_SOURCE_DIR}/TPL/Eigen3)
include_directories(${CMAKE_SOURCE_DIR}/TPL/xtensor/include)
include_directories(${CMAKE_SOURCE_DIR}/TPL/yaml/include)
include_directories(${CMAKE_SOURCE_DIR}/TPL/opencv/include/opencv4)
link_directories(${CMAKE_SOURCE_DIR}/TPL/jsoncpp/lib)
link_directories(${CMAKE_SOURCE_DIR}/TPL/opencv/lib)
link_directories(${CMAKE_SOURCE_DIR}/TPL/yaml/lib)
link_directories(${CMAKE_SOURCE_DIR}/Bin/lib)


add_executable(ObstacleTool ${CMAKE_SOURCE_DIR}/GeomTool/Obstacle/gen.cpp)
add_executable(SeTool ${CMAKE_SOURCE_DIR}/GeomTool/Se/main.cpp)

add_subdirectory(
        pybind11
)
# message("all src files : ${ALL_MY_SRC}")
# 导入pybind11库
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development)
# 打印找到的 Python 解释器路径以确保正确查找
message(STATUS "Found Python3 interpreter: ${Python3_EXECUTABLE}")
execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/PythonTool/find_numpy_include.py
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
# 打印找到的 NumPy 头文件路径以确保正确查找
message(STATUS "Found NumPy include dir: ${NUMPY_INCLUDE_DIR}")
include_directories( ${NUMPY_INCLUDE_DIR})
# SET(Python3_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")

add_executable(ParseTool main.cpp
${ALL_MY_SRC}
)
target_link_libraries(ParseTool -ljsoncpp 
-lopencv_core -lopencv_features2d -lopencv_flann -lopencv_imgproc -lopencv_calib3d 
${Python3_LIBRARIES} pybind11::embed
-lyaml-cpp
)
