cmake_minimum_required(VERSION 3.20)
project(cinematica_industrial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(glfw3 CONFIG REQUIRED)
find_package(glad  CONFIG REQUIRED)
find_package(glm   CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(cinematica_industrial
    src/main.cpp
    src/camera3d.cpp
    src/renderer3d.cpp
    src/animation.cpp
    src/manipulators/cartesian.cpp
    src/manipulators/polar.cpp
)

target_link_libraries(cinematica_industrial
    PRIVATE
        glfw
        glad::glad
        glm::glm
        imgui::imgui
)

target_include_directories(cinematica_industrial PRIVATE src)

if(MSVC)
    target_compile_options(cinematica_industrial PRIVATE /W4 /permissive-)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT cinematica_industrial)
endif()
