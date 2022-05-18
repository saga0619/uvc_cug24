PROJECT (uvc_example)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

add_compile_options(-std=c++17 -Ofast -fpermissive)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)

  find_package(OpenCVPkg REQUIRED
    COMPONENTS
      opencv_core
      opencv_highgui
  )

  find_package(LibUVC REQUIRED
    COMPONENTS
      LibUSB
      JpegPkg)


include_directories(
    include
    ${LibUVC_INCLUDE_DIR}
    )

add_executable(uvc_example main.cpp)

  target_link_libraries(uvc_example
    PRIVATE
      ${LibUVC_LIBRARY}
      opencv_core
      opencv_highgui
  )