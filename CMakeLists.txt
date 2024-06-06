cmake_minimum_required(VERSION 3.22)
project(ppBenchmark)

set(CMAKE_CXX_STANDARD 17)

add_executable(ppBenchmark
        main.cpp
        RRT_star.cpp
        RRT_star.h
        Environment.cpp
        Environment.h
        WebSocketServer.cpp
        WebSocketServer.h
        stoppableThread.cpp
        stoppableThread.h
        CustomPlanner.cpp
        CustomPlanner.h
)

include_directories(/usr/include/websocketpp /usr/include/nlohmann)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ppBenchmark ${OpenCV_LIBS})

set(octomap_DIR "/opt/ros/humble/share/octomap")
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(ppBenchmark ${OCTOMAP_LIBRARIES})

set(dynamicEDT3D_DIR "/opt/ros/humble/share/dynamicEDT3D")
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
target_link_libraries(ppBenchmark dynamicedt3d)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
target_link_libraries(ppBenchmark ${OMPL_LIBRARIES})

find_package(websocketpp REQUIRED)
include_directories(${WEBSOCKETPP_INCLUDE_DIR})
target_link_libraries(ppBenchmark ${WEBSOCKETPP_LIBRARIES})

find_package(nlohmann_json REQUIRED)
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(ppBenchmark nlohmann_json::nlohmann_json)
