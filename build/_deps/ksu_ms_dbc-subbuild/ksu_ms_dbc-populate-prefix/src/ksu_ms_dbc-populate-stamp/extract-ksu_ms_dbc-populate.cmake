<<<<<<< HEAD
# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

=======
>>>>>>> aa20f60 (include all shit)
# Make file names absolute:
#
get_filename_component(filename "/home/klee/Github/VCU/build/_deps/ksu_ms_dbc-subbuild/ksu_ms_dbc-populate-prefix/src/can_lib.tar.gz" ABSOLUTE)
get_filename_component(directory "/home/klee/Github/VCU/build/_deps/ksu_ms_dbc-src" ABSOLUTE)

<<<<<<< HEAD
message(VERBOSE "extracting...
     src='${filename}'
     dst='${directory}'"
)

if(NOT EXISTS "${filename}")
  message(FATAL_ERROR "File to extract does not exist: '${filename}'")
=======
message(STATUS "extracting...
     src='${filename}'
     dst='${directory}'")

if(NOT EXISTS "${filename}")
  message(FATAL_ERROR "error: file to extract does not exist: '${filename}'")
>>>>>>> aa20f60 (include all shit)
endif()

# Prepare a space for extracting:
#
set(i 1234)
while(EXISTS "${directory}/../ex-ksu_ms_dbc-populate${i}")
  math(EXPR i "${i} + 1")
endwhile()
set(ut_dir "${directory}/../ex-ksu_ms_dbc-populate${i}")
file(MAKE_DIRECTORY "${ut_dir}")

# Extract it:
#
<<<<<<< HEAD
message(VERBOSE "extracting... [tar xfz]")
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xfz ${filename} 
  WORKING_DIRECTORY ${ut_dir}
  RESULT_VARIABLE rv
)

if(NOT rv EQUAL 0)
  message(VERBOSE "extracting... [error clean up]")
  file(REMOVE_RECURSE "${ut_dir}")
  message(FATAL_ERROR "Extract of '${filename}' failed")
=======
message(STATUS "extracting... [tar xfz]")
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xfz ${filename}
  WORKING_DIRECTORY ${ut_dir}
  RESULT_VARIABLE rv)

if(NOT rv EQUAL 0)
  message(STATUS "extracting... [error clean up]")
  file(REMOVE_RECURSE "${ut_dir}")
  message(FATAL_ERROR "error: extract of '${filename}' failed")
>>>>>>> aa20f60 (include all shit)
endif()

# Analyze what came out of the tar file:
#
<<<<<<< HEAD
message(VERBOSE "extracting... [analysis]")
=======
message(STATUS "extracting... [analysis]")
>>>>>>> aa20f60 (include all shit)
file(GLOB contents "${ut_dir}/*")
list(REMOVE_ITEM contents "${ut_dir}/.DS_Store")
list(LENGTH contents n)
if(NOT n EQUAL 1 OR NOT IS_DIRECTORY "${contents}")
  set(contents "${ut_dir}")
endif()

# Move "the one" directory to the final directory:
#
<<<<<<< HEAD
message(VERBOSE "extracting... [rename]")
=======
message(STATUS "extracting... [rename]")
>>>>>>> aa20f60 (include all shit)
file(REMOVE_RECURSE ${directory})
get_filename_component(contents ${contents} ABSOLUTE)
file(RENAME ${contents} ${directory})

# Clean up:
#
<<<<<<< HEAD
message(VERBOSE "extracting... [clean up]")
file(REMOVE_RECURSE "${ut_dir}")

message(VERBOSE "extracting... done")
=======
message(STATUS "extracting... [clean up]")
file(REMOVE_RECURSE "${ut_dir}")

message(STATUS "extracting... done")
>>>>>>> aa20f60 (include all shit)
