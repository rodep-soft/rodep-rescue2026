# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/root/microros_ws/firmware/dev_ws/ament/ament_cmake/ament_cmake_vendor_package/test/depender"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-build"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/install"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/tmp"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src"
  "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/root/microros_ws/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp${cfgdir}") # cfgdir has leading slash
endif()
