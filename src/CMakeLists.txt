# SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
# SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
#
# SPDX-License-Identifier: Apache-2.0

add_library(paramextract STATIC IMPORTED)
set_property(TARGET paramextract PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/ParameterExtraction.yrcLib")
target_include_directories(paramextract INTERFACE ${CMAKE_CURRENT_LIST_DIR})

add_library(motoros_platform_lib STATIC IMPORTED)
set_property(TARGET motoros_platform_lib PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/MotoROS_PlatformLib.yrcLib")
target_include_directories(motoros_platform_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_compile_features(motoros_platform_lib INTERFACE c_std_99)

add_executable(${PROJECT_NAME}
  main.c
  ActionServer_FJT.c
  CommunicationExecutor.c
  ConfigFile.c
  ControllerStatusIO.c
  CtrlGroup.c
  Debug.c
  ErrorHandling.c
  FauxCommandLineArgs.c
  FileUtilityFunctions.c
  InformCheckerAndGenerator.c
  MemoryAllocation.c
  MotionControl.c
  PositionMonitor.c
  Quaternion_Conversion.c
  ServiceQueueTrajPoint.c
  ServiceReadWriteIO.c
  ServiceResetError.c
  ServiceSelectMotionTool.c
  ServiceStartPointQueueMode.c
  ServiceStartTrajMode.c
  ServiceStopTrajMode.c
  Tests_CtrlGroup.c
)
target_link_libraries(${PROJECT_NAME} PRIVATE microros motoros_platform_lib paramextract)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<BOOL:${MOTOROS2_ENABLE_MEMORY_TRACING}>:MOTOROS2_MEM_TRACE_ENABLE>
  $<$<BOOL:${MOTOROS2_BUILD_TESTING}>:MOTOROS2_TESTING_ENABLE>)
# force main binary to be linked in root of build dir
set_target_properties(${PROJECT_NAME}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
