cmake_minimum_required(VERSION 3.8)
project(dropbear_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Move folders to the shared folder
install(
  DIRECTORY
    launch meshes rviz urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
