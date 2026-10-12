cmake_minimum_required(VERSION 3.13)

project(inav C CXX ASM)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Get git revision for version information
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_REV)
if(NOT GIT_REV)
    set(GIT_REV "UNKNOWN")
endif()

# Determine if this is a release build
string(REGEX MATCH "refs/tags/release" IS_TAG "${GIT_REFSPEC}")
if(IS_TAG)
    set(IS_RELEASE_BUILD TRUE)
else()
    set(IS_RELEASE_BUILD FALSE)
endif()

# Source and library directories
set(MAIN_SRC_DIR "${CMAKE_SOURCE_DIR}/src/main")
set(MAIN_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")

# Include main cmake configuration
include(main)

# Add the main source directory
add_subdirectory(src)