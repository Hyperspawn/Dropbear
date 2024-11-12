# Install script for directory: /home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/gateway/dropbear_detailed_urdf_wcontrollers/install/dropbear_detailed_urdf")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dropbear_detailed_urdf" TYPE PROGRAM FILES
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/dropbear_detailed_urdf/hand_trajectory_publisher.py"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/dropbear_detailed_urdf/leg_trajectory_publisher.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE DIRECTORY FILES
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/launch"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/meshes"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/rviz"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/urdf"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dropbear_detailed_urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dropbear_detailed_urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf/environment" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf/environment" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_index/share/ament_index/resource_index/packages/dropbear_detailed_urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf/cmake" TYPE FILE FILES
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_core/dropbear_detailed_urdfConfig.cmake"
    "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/ament_cmake_core/dropbear_detailed_urdfConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dropbear_detailed_urdf" TYPE FILE FILES "/home/gateway/dropbear_detailed_urdf_wcontrollers/src/dropbear_detailed_urdf/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/gateway/dropbear_detailed_urdf_wcontrollers/build/dropbear_detailed_urdf/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
