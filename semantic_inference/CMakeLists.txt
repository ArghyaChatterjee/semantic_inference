cmake_minimum_required(VERSION 3.16)
project(semantic_inference VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-deprecated-declarations)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(SEMANTIC_INFERENCE_USE_TRT "Build with TensorRT" ON)
option(SEMANTIC_INFERENCE_BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(config_utilities REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA)

# Manually find TensorRT libraries
find_library(TENSORRT_LIBRARY nvinfer HINTS /home/arghya/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin HINTS /home/arghya/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib)
find_library(TENSORRT_ONNX_LIBRARY nvonnxparser HINTS /home/arghya/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib)

if(NOT TENSORRT_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY OR NOT TENSORRT_ONNX_LIBRARY)
    message(FATAL_ERROR "Could not find all required TensorRT libraries")
endif()

configure_file(
  cmake/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/semantic_inference_config.h
)

add_library(${PROJECT_NAME}
  src/image_recolor.cpp src/image_rotator.cpp src/image_utilities.cpp
  src/logging.cpp src/model_config.cpp src/segmenter.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC config_utilities::config_utilities ${OpenCV_LIBRARIES}
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>  # Include binary directory
         ${OpenCV_INCLUDE_DIRS}
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Handle TensorRT
if(SEMANTIC_INFERENCE_USE_TRT)
    target_link_libraries(${PROJECT_NAME}
      PRIVATE ${TENSORRT_LIBRARY} ${TENSORRT_PLUGIN_LIBRARY} ${TENSORRT_ONNX_LIBRARY} ${CUDA_LIBRARIES}
    )
    target_include_directories(${PROJECT_NAME} PRIVATE /home/arghya/TensorRT-8.6.1.6-cuda-12.1/TensorRT-8.6.1.6/include ${CUDA_INCLUDE_DIRS})
    target_sources(${PROJECT_NAME} PRIVATE src/trt_utilities.cpp src/model.cpp)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_TENSORRT=1)
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_TENSORRT=0)
endif()

add_executable(demo_segmentation app/demo_segmentation.cpp)
target_link_libraries(demo_segmentation ${PROJECT_NAME})

if(SEMANTIC_INFERENCE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
  TARGETS ${PROJECT_NAME} demo_segmentation
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
