# cmake/libdeps.cmake
include(FetchContent)

# Where all external deps get downloaded
set(DEPS_DIR "${CMAKE_BINARY_DIR}/external" CACHE PATH "Directory for downloaded deps")

# Fetch git repo and add its sources directly to a target
macro(fetch_and_add_git_lib TARGET_NAME LIB_NAME GIT_URL GIT_TAG)
    message(STATUS "fetch_and_add_git_lib: ${LIB_NAME} from ${GIT_URL} (tag ${GIT_TAG})")

    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "fetch_and_add_git_lib: target ${TARGET_NAME} does not exist (LIB_NAME=${LIB_NAME})")
    endif()

    set(_src_dir "${DEPS_DIR}/${LIB_NAME}")

    FetchContent_Declare(${LIB_NAME}
        GIT_REPOSITORY ${GIT_URL}
        GIT_TAG        ${GIT_TAG}
        SOURCE_DIR     ${_src_dir}
    )
    FetchContent_MakeAvailable(${LIB_NAME})

    file(GLOB_RECURSE ${LIB_NAME}_SOURCES
        CONFIGURE_DEPENDS
        "${_src_dir}/*.c"
        "${_src_dir}/*.cpp"
    )

    if(${LIB_NAME}_SOURCES)
        target_sources(${TARGET_NAME} PRIVATE ${${LIB_NAME}_SOURCES})
    endif()

    target_include_directories(${TARGET_NAME} PRIVATE
        "${_src_dir}"
        "${_src_dir}/src"
    )
endmacro()


macro(fetch_and_add_url_lib LIB_NAME LIB_URL LIB_PATH)
    include(FetchContent)
    FetchContent_Declare(${LIB_NAME}
            URL ${LIB_URL}
            SOURCE_DIR ${_src_dir}
    )
    FetchContent_Populate(${LIB_NAME})
    string(TOLOWER ${LIB_NAME} LIB_NAME_LOWER)
    set(LIB_ROOT "${${LIB_NAME_LOWER}_SOURCE_DIR}/${LIB_PATH}")

    # Check if we can find the library.
    if(NOT EXISTS "${LIB_ROOT}")
        message(STATUS "Could not find the directory for library '${LIB_ROOT}' -- ignoring (its possible that the library is not used for the target you are calling) !!!!!")
    else()
        import_arduino_library(${LIB_NAME} ${LIB_ROOT} ${ARGN})
    endif(NOT EXISTS "${LIB_ROOT}")

endmacro(fetch_and_add_url_lib)