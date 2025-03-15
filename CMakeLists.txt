project(lidar)

set(GLFW_PATH "/home/ethans/libs/glfw/include/GLFW")

include_directories(${GLFW_PATH}/include)


find_package(OpenGL REQUIRED)
find_package(wpilib REQUIRED)

add_executable(main main.cpp)

target_link_libraries(main ntcore glfw OpenGL::GL)
