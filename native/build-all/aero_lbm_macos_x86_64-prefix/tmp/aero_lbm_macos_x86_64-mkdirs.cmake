# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/build-macos_x86_64"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/tmp"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/src/aero_lbm_macos_x86_64-stamp"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/src"
  "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/src/aero_lbm_macos_x86_64-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/src/aero_lbm_macos_x86_64-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/firedoge/Projects/fno/aerodynamics4mc/fabric-mod/native/build-all/aero_lbm_macos_x86_64-prefix/src/aero_lbm_macos_x86_64-stamp${cfgdir}") # cfgdir has leading slash
endif()
