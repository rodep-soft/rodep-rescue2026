# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/root/microros_ws/firmware/dev_ws/ament/ament_cmake/ament_cmake_vendor_package/test/exlib_good"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src/exlib_good-build"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/install"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/tmp"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src/exlib_good-stamp"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src/exlib_good-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src/exlib_good-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/exlib_good-prefix/src/exlib_good-stamp${cfgdir}") # cfgdir has leading slash
endif()
