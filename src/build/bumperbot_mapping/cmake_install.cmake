# Install script for directory: /home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/install/bumperbot_mapping")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/bumperbot_mapping-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_python/bumperbot_mapping/bumperbot_mapping.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/bumperbot_mapping" TYPE DIRECTORY FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/bumperbot_mapping/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/install/bumperbot_mapping/local/lib/python3.10/dist-packages/bumperbot_mapping"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/include")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping" TYPE EXECUTABLE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/mapping_with_known_poses")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping/mapping_with_known_poses")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE DIRECTORY FILES
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/config"
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/launch"
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/maps"
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/rviz"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bumperbot_mapping" TYPE PROGRAM FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/bumperbot_mapping/mapping_with_known_poses.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/bumperbot_mapping")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/bumperbot_mapping")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/environment" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_index/share/ament_index/resource_index/packages/bumperbot_mapping")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping/cmake" TYPE FILE FILES
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_core/bumperbot_mappingConfig.cmake"
    "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/ament_cmake_core/bumperbot_mappingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bumperbot_mapping" TYPE FILE FILES "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/bumperbot_mapping/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ray/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization/Section10_SLAM/bumperbot_ws/src/build/bumperbot_mapping/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
