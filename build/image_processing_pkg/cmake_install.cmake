# Install script for directory: /home/odroid/artbot_ws/src/image_processing_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/odroid/artbot_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/msg" TYPE FILE FILES "/home/odroid/artbot_ws/src/image_processing_pkg/msg/ProcessedImage.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/action" TYPE FILE FILES "/home/odroid/artbot_ws/src/image_processing_pkg/action/take_selfie.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/msg" TYPE FILE FILES
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieAction.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieActionGoal.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieActionResult.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieActionFeedback.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieGoal.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieResult.msg"
    "/home/odroid/artbot_ws/devel/share/image_processing_pkg/msg/take_selfieFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/cmake" TYPE FILE FILES "/home/odroid/artbot_ws/build/image_processing_pkg/catkin_generated/installspace/image_processing_pkg-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/odroid/artbot_ws/devel/include/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/odroid/artbot_ws/devel/share/roseus/ros/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/odroid/artbot_ws/devel/share/common-lisp/ros/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/odroid/artbot_ws/devel/share/gennodejs/ros/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/odroid/artbot_ws/devel/lib/python2.7/dist-packages/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/odroid/artbot_ws/devel/lib/python2.7/dist-packages/image_processing_pkg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/odroid/artbot_ws/build/image_processing_pkg/catkin_generated/installspace/image_processing_pkg.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/cmake" TYPE FILE FILES "/home/odroid/artbot_ws/build/image_processing_pkg/catkin_generated/installspace/image_processing_pkg-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg/cmake" TYPE FILE FILES
    "/home/odroid/artbot_ws/build/image_processing_pkg/catkin_generated/installspace/image_processing_pkgConfig.cmake"
    "/home/odroid/artbot_ws/build/image_processing_pkg/catkin_generated/installspace/image_processing_pkgConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/image_processing_pkg" TYPE FILE FILES "/home/odroid/artbot_ws/src/image_processing_pkg/package.xml")
endif()

