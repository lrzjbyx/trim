cmake_minimum_required(VERSION 3.23)
project(trim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.4.2")
set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.7.0")
set(Boost_DIR "/opt/homebrew/Cellar/boost/1.80.0")
set(NUMCPP_INCLUDES "/opt/homebrew/Cellar/numcpp/2.8.0")

find_package(OpenCV)
find_package(Boost 1.80.0 COMPONENTS system filesystem REQUIRED)
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED
        )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


add_executable(trim main.cpp trim.cpp)
target_link_libraries(trim
        Qt::Core
        Qt::Gui
        Qt::Widgets
        ${OpenCV_LIBS}
        ${Boost_LIBRARsIES}
        )

target_include_directories(trim PRIVATE
        ${NUMCPP_INCLUDES}
        )