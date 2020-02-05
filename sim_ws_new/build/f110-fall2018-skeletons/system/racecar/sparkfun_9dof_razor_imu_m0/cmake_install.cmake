# Install script for directory: /home/nvidia/sandbox/sim_ws/src/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/sandbox/sim_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nvidia/sandbox/sim_ws/build/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/catkin_generated/installspace/sparkfun_9dof_razor_imu_m0.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sparkfun_9dof_razor_imu_m0/cmake" TYPE FILE FILES
    "/home/nvidia/sandbox/sim_ws/build/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/catkin_generated/installspace/sparkfun_9dof_razor_imu_m0Config.cmake"
    "/home/nvidia/sandbox/sim_ws/build/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/catkin_generated/installspace/sparkfun_9dof_razor_imu_m0Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sparkfun_9dof_razor_imu_m0" TYPE FILE FILES "/home/nvidia/sandbox/sim_ws/src/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0" TYPE EXECUTABLE FILES "/home/nvidia/sandbox/sim_ws/devel/lib/sparkfun_9dof_razor_imu_m0/driver_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node"
         OLD_RPATH "/home/nvidia/sandbox/sim_ws/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sparkfun_9dof_razor_imu_m0/driver_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nvidia/sandbox/sim_ws/devel/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so"
         OLD_RPATH "/home/nvidia/sandbox/sim_ws/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsparkfun_9dof_razor_imu_m0_driver_nodelet.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sparkfun_9dof_razor_imu_m0" TYPE DIRECTORY FILES "/home/nvidia/sandbox/sim_ws/src/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/include/sparkfun_9dof_razor_imu_m0/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sparkfun_9dof_razor_imu_m0" TYPE FILE FILES "/home/nvidia/sandbox/sim_ws/src/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/nodelet_plugin.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sparkfun_9dof_razor_imu_m0/launch" TYPE DIRECTORY FILES "/home/nvidia/sandbox/sim_ws/src/f110-fall2018-skeletons/system/racecar/sparkfun_9dof_razor_imu_m0/launch/")
endif()

