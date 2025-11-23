# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/klee/Github/VCU/build/_deps/sdfat-src")
  file(MAKE_DIRECTORY "/home/klee/Github/VCU/build/_deps/sdfat-src")
endif()
file(MAKE_DIRECTORY
  "/home/klee/Github/VCU/build/_deps/sdfat-build"
  "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix"
  "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/tmp"
  "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/src/sdfat-populate-stamp"
  "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/src"
  "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/src/sdfat-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/src/sdfat-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/klee/Github/VCU/build/_deps/sdfat-subbuild/sdfat-populate-prefix/src/sdfat-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
