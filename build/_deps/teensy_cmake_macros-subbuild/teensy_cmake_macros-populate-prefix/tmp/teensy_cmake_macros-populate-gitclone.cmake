<<<<<<< HEAD
# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

if(EXISTS "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt" AND EXISTS "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitinfo.txt" AND
  "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt" IS_NEWER_THAN "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitinfo.txt")
  message(VERBOSE
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt'"
  )
  return()
endif()

# Even at VERBOSE level, we don't want to see the commands executed, but
# enabling them to be shown for DEBUG may be useful to help diagnose problems.
cmake_language(GET_MESSAGE_LOG_LEVEL active_log_level)
if(active_log_level MATCHES "DEBUG|TRACE")
  set(maybe_show_command COMMAND_ECHO STDOUT)
else()
  set(maybe_show_command "")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
=======

if(NOT "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitinfo.txt" IS_NEWER_THAN "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
  RESULT_VARIABLE error_code
  )
>>>>>>> aa20f60 (include all shit)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
<<<<<<< HEAD
    COMMAND "/usr/bin/git"
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/newdigate/teensy-cmake-macros.git" "teensy_cmake_macros-src"
    WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(NOTICE "Had to git clone more than once: ${number_of_tries} times.")
=======
    COMMAND "/usr/bin/git"  clone --no-checkout "https://github.com/newdigate/teensy-cmake-macros.git" "teensy_cmake_macros-src"
    WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
>>>>>>> aa20f60 (include all shit)
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/newdigate/teensy-cmake-macros.git'")
endif()

execute_process(
<<<<<<< HEAD
  COMMAND "/usr/bin/git"
          checkout "main" --
  WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
=======
  COMMAND "/usr/bin/git"  checkout main --
  WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
  RESULT_VARIABLE error_code
  )
>>>>>>> aa20f60 (include all shit)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'main'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
<<<<<<< HEAD
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
=======
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src"
    RESULT_VARIABLE error_code
    )
>>>>>>> aa20f60 (include all shit)
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
<<<<<<< HEAD
  COMMAND ${CMAKE_COMMAND} -E copy "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitinfo.txt" "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt'")
endif()
=======
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitinfo.txt"
    "/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/klee/Github/VCU/build/_deps/teensy_cmake_macros-subbuild/teensy_cmake_macros-populate-prefix/src/teensy_cmake_macros-populate-stamp/teensy_cmake_macros-populate-gitclone-lastrun.txt'")
endif()

>>>>>>> aa20f60 (include all shit)
