# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.18.4)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(cmake_declare_and_fetch-populate NONE)

include(ExternalProject)
ExternalProject_Add(cmake_declare_and_fetch-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/newdigate/cmake-declare-and-fetch.git" "GIT_TAG" "main"
                    SOURCE_DIR          "/home/klee/Github/VCU/build/_deps/cmake_declare_and_fetch-src"
                    BINARY_DIR          "/home/klee/Github/VCU/build/_deps/cmake_declare_and_fetch-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


