# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(argusCamera
  CATEGORY device
  TYPE argusCameraDriver
  INCLUDE argusCameraDriver.h
  EXTRA_CONFIG WRAPPER=frameGrabber_nws_yarp
  GENERATE_PARSER
  DEFAULT ON
)

if(ENABLE_argusCamera)
  option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Generate the param parser for argusCamera device" OFF)
  yarp_add_plugin(yarp_argusCamera)

  target_sources(yarp_argusCamera
    PRIVATE
      argusCameraDriver.h
      argusCameraDriver.cpp
      argusCameraDriver_ParamsParser.h
      argusCameraDriver_ParamsParser.cpp
  )

  list(APPEND OPENCV_DEPS opencv_core
                          opencv_video
                          opencv_imgproc)

  if (CUDA_FOUND AND OpenCV_CUDA_VERSION AND TRY_ACTIVATE_CUDA)
    target_compile_definitions(yarp_argusCamera PUBLIC -DUSE_CUDA)
  endif()


  include_directories(
    ${ARGUS_INCLUDE_DIRS}
  )

  target_link_libraries(yarp_argusCamera
    PUBLIC
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      YARP::YARP_cv
    PRIVATE
      ${ARGUS_LIBRARIES}
      ${OPENCV_DEPS}
  )

  yarp_install(
    TARGETS yarp_argusCamera
    EXPORT yarp-device-argus
    COMPONENT yarp-device-argus
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set_property(TARGET yarp_argusCamera PROPERTY FOLDER "Plugins/Device")
endif()