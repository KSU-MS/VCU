# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

<<<<<<< HEAD
cmake_minimum_required(VERSION 3.31.6)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})
=======
cmake_minimum_required(VERSION 3.18.4)
>>>>>>> aa20f60 (include all shit)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(ksu_ms_dbc-populate NONE)

<<<<<<< HEAD


include(ExternalProject)
ExternalProject_Add(ksu_ms_dbc-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "TRUE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "https://github.com/KSU-MS/ksu-ms-dbc/releases/latest/download/can_lib.tar.gz"
=======
include(ExternalProject)
ExternalProject_Add(ksu_ms_dbc-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/KSU-MS/ksu-ms-dbc/releases/latest/download/can_lib.tar.gz"
>>>>>>> aa20f60 (include all shit)
                    SOURCE_DIR          "/home/klee/Github/VCU/build/_deps/ksu_ms_dbc-src"
                    BINARY_DIR          "/home/klee/Github/VCU/build/_deps/ksu_ms_dbc-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
<<<<<<< HEAD
                    USES_TERMINAL_PATCH     YES
=======
>>>>>>> aa20f60 (include all shit)
)


