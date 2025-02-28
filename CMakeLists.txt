cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

# add_compile_options(-std=gnu++11)
# 使用更规范的方式设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 可选：启用GNU扩展

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g")

# set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

#添加C++ ABI定义以解决未定义引用问题
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)  # 或 0 视情况而定

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
find_package(jsoncpp REQUIRED)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
set(dirPino "third_party/pinocchio")
set(dirQuill "third_party/quill")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")

# third_party pinocchio, boost can be used, but need to installl urdfdom and jsoncpp
link_directories(third_party/pinocchio)
link_directories(third_party/quill)
link_directories(third_party/quill/quill/bundled)
link_directories(third_party/boost)
link_directories("/usr/lib/x86_64-linux-gnu/libjsoncpp.so")
link_directories("/usr/lib/x86_64-linux-gnu/liburdfdom_model.so")
link_directories("/usr/lib/x86_64-linux-gnu/libconsole_bridge.so")
link_directories("/usr/lib/x86_64-linux-gnu/libtinyxml.so")

set(allDir ${dirPino} ${dirQuill})
set(allInc ${allDir} ${incFmt} ${incBoost})


include_directories (${QPOASES_INLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})

include_directories(${allInc})
include_directories(
    include
    third_party/alglib/src
	/usr/include
	/usr/include/jsoncpp
)

file(GLOB_RECURSE alglib_files "${CMAKE_CURRENT_SOURCE_DIR}/third_party/alglib/src/**.cpp")
add_library(alglib_sdk ${alglib_files})

add_executable(nlctest
    nlctest.cpp
)
target_link_libraries(nlctest
    alglib_sdk
)

set(SOURCES src/pino_kin_dyn.cpp src/wbc_priority.cpp src/priority_tasks.cpp src/useful_math.cpp)

#链接库
set(corLibs pinocchio quill)
#根据架构，处理连接库
set(sysCoreLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

add_library(leg_controller	src/leg_controller.cpp)
add_library(swing_leg_controller	src/swing_leg_controller.cpp)
add_library(stance_leg_controller	src/stance_leg_controller.cpp)
add_library(control	src/control.cpp)
add_library(gait_generator	src/gait_generator.cpp)
add_library(model	src/model.cpp)
add_library(opmath	src/opmath.cpp)
add_library(adaptivefootPlace	src/adaptivefootPlace.cpp)
add_library(core  ${SOURCES})

target_link_libraries(model PUBLIC raisim::raisim pthread opmath)
target_include_directories(model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(core ${sysCoreLibs} pthread qpOASES libjsoncpp.so libconsole_bridge.so liburdfdom_model.so libtinyxml.so)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(opmath PUBLIC pthread)
target_include_directories(opmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(leg_controller PUBLIC raisim::raisim pthread model opmath swing_leg_controller stance_leg_controller control)
target_include_directories(leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gait_generator PUBLIC raisim::raisim pthread model opmath)
target_include_directories(gait_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(stance_leg_controller PUBLIC raisim::raisim pthread model opmath qpOASES)
target_include_directories(stance_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(swing_leg_controller PUBLIC raisim::raisim pthread model opmath control adaptivefootPlace)
target_include_directories(swing_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(control PUBLIC raisim::raisim pthread opmath)
target_include_directories(control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(qptest qptest.cpp)
target_link_libraries(qptest qpOASES)

add_executable(Biped main.cpp)
target_link_libraries(Biped PUBLIC core raisim::raisim pthread leg_controller model opmath swing_leg_controller stance_leg_controller gait_generator control adaptivefootPlace)
target_include_directories(Biped PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
