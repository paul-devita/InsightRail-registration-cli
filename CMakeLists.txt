cmake_minimum_required(VERSION 3.24)

# locate vcpkg
#-----------------------------------------------------------------------------------------------------------------------
if (NOT VCPKG_ROOT AND DEFINED ENV{VCPKG_ROOT})
    set(VCPKG_ROOT "$ENV{VCPKG_ROOT}")
endif()

if (NOT VCPKG_ROOT)
    message(FATAL_ERROR
            "please specify -DVCPKG_ROOT=/path/to/vcpkg"
    )
endif ()

file(TO_CMAKE_PATH "${VCPKG_ROOT}" VCPKG_ROOT)

set(_VCPKG_TOOLCHAIN
        "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
)

if (NOT EXISTS "${_VCPKG_TOOLCHAIN}")
    message(FATAL_ERROR
            "Cannot find vcpkg toolchain: ${_VCPKG_TOOLCHAIN}"
    )
endif ()

# select platform triplet
#-----------------------------------------------------------------------------------------------------------------------
if(NOT DEFINED VCPKG_TARGET_TRIPLET)
    if(WIN32)
        set(VCPKG_TARGET_TRIPLET "x64-windows")
    elseif(APPLE)
        if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
            set(VCPKG_TARGET_TRIPLET "arm64-osx")
        else()
            set(VCPKG_TARGET_TRIPLET "x64-osx")
        endif()
    elseif(UNIX)
        set(VCPKG_TARGET_TRIPLET "x64-linux")
    endif()
endif()

# set toolchain
#-----------------------------------------------------------------------------------------------------------------------
set(CMAKE_TOOLCHAIN_FILE
        "${_VCPKG_TOOLCHAIN}"
        CACHE FILEPATH "vcpkg toolchain"
)

# init project
#-----------------------------------------------------------------------------------------------------------------------
project(registration_cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# download open3d (Debug / Release)
# ------------------------------------------------------------
if(NOT WIN32)
    message(FATAL_ERROR
            "This Open3D setup currently supports Windows x64 only."
    )
endif()

# These prebuilt Open3D packages contain one build variant each.
# Use separate single-config build directories (e.g. Ninja).
get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

if(IS_MULTI_CONFIG)
    message(FATAL_ERROR
            "Use a single-config generator such as Ninja with "
            "separate Debug and Release build directories."
    )
endif()

# Default to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
            "Build configuration" FORCE)
endif()

set(OPEN3D_VERSION "0.20.0")

# Select package based on compilation configuration
if(CMAKE_BUILD_TYPE STREQUAL "Debug")

    set(OPEN3D_VARIANT "debug")
    set(OPEN3D_ARCHIVE_SUFFIX "-dbg")
    set(OPEN3D_SHA256
            "e4c5e04669f80c9464f2fcab61cea78ab9c18080e94091b7fcb24c53b8c87801"
    )

elseif(CMAKE_BUILD_TYPE STREQUAL "Release"
        OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")

    set(OPEN3D_VARIANT "release")
    set(OPEN3D_ARCHIVE_SUFFIX "")
    set(OPEN3D_SHA256
            "ae2843b64da4d82e0cabf82462a9fda12921ea5be9a9d593f14a7a21a44a97fe"
    )

else()
    message(FATAL_ERROR
            "Unsupported build configuration: ${CMAKE_BUILD_TYPE}"
    )
endif()

message(STATUS
        "Building ${CMAKE_BUILD_TYPE} with ${OPEN3D_VARIANT} Open3D"
)

# Archive information
set(OPEN3D_ARCHIVE_NAME
        "open3d-devel-windows-amd64-${OPEN3D_VERSION}${OPEN3D_ARCHIVE_SUFFIX}.zip"
)

set(OPEN3D_URL
        "https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/${OPEN3D_ARCHIVE_NAME}"
)

# Directories
set(OPEN3D_DOWNLOAD_DIR
        "${CMAKE_BINARY_DIR}/_deps/downloads"
)

set(OPEN3D_ROOT
        "${CMAKE_BINARY_DIR}/_deps/open3d-${OPEN3D_VERSION}-${OPEN3D_VARIANT}"
)

set(OPEN3D_ARCHIVE
        "${OPEN3D_DOWNLOAD_DIR}/${OPEN3D_ARCHIVE_NAME}"
)

file(MAKE_DIRECTORY "${OPEN3D_DOWNLOAD_DIR}")

# Download and verify (reuses a valid cached archive)
file(DOWNLOAD
        "${OPEN3D_URL}"
        "${OPEN3D_ARCHIVE}"
        EXPECTED_HASH "SHA256=${OPEN3D_SHA256}"
        TLS_VERIFY ON
        SHOW_PROGRESS
        STATUS _download_status
)

list(GET _download_status 0 _download_code)
list(GET _download_status 1 _download_message)

if(NOT _download_code EQUAL 0)
    message(FATAL_ERROR
            "Open3D download failed: ${_download_message}"
    )
endif()

# Check whether the package is already extracted
file(GLOB_RECURSE OPEN3D_CONFIG_FILES
        LIST_DIRECTORIES FALSE
        "${OPEN3D_ROOT}/Open3DConfig.cmake"
)

if(NOT OPEN3D_CONFIG_FILES)
    message(STATUS "Extracting ${OPEN3D_VARIANT} Open3D...")

    file(MAKE_DIRECTORY "${OPEN3D_ROOT}")

    file(ARCHIVE_EXTRACT
            INPUT "${OPEN3D_ARCHIVE}"
            DESTINATION "${OPEN3D_ROOT}"
    )

    # Search again after extraction
    file(GLOB_RECURSE OPEN3D_CONFIG_FILES
            LIST_DIRECTORIES FALSE
            "${OPEN3D_ROOT}/Open3DConfig.cmake"
    )
endif()

# Validate configuration discovery
list(LENGTH OPEN3D_CONFIG_FILES OPEN3D_CONFIG_COUNT)

if(OPEN3D_CONFIG_COUNT EQUAL 0)
    message(FATAL_ERROR
            "Open3DConfig.cmake not found under ${OPEN3D_ROOT}"
    )
elseif(OPEN3D_CONFIG_COUNT GREATER 1)
    message(FATAL_ERROR
            "Multiple Open3D configurations found: ${OPEN3D_CONFIG_FILES}"
    )
endif()

# Get actual configuration path
list(GET OPEN3D_CONFIG_FILES 0 OPEN3D_CONFIG_FILE)

get_filename_component(OPEN3D_CONFIG_DIR
        "${OPEN3D_CONFIG_FILE}" DIRECTORY
)

# Set package path for find_package()
set(Open3D_DIR "${OPEN3D_CONFIG_DIR}"
        CACHE PATH "Open3D configuration directory" FORCE
)

message(STATUS
        "Found ${OPEN3D_VARIANT} Open3D config: ${Open3D_DIR}"
)

# Locate Open3D
find_package(Open3D CONFIG REQUIRED)

# find dependencies
#-----------------------------------------------------------------------------------------------------------------------
find_package(TIFF REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

find_package(Open3D CONFIG REQUIRED
        PATHS "${OPEN3D_ROOT}/CMake"
        NO_DEFAULT_PATH
)

# build executable
#-----------------------------------------------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(registration_cli
        ${SOURCES}
        ${HEADERS}
)

# set to ON to enable debug messages in ImageProcessing functionalities
option(ENABLE_DEBUG_LOG "Enable debug logging" ON)

if (ENABLE_DEBUG_LOG)
    target_compile_definitions(registration_cli
            PRIVATE IRTOOLS_IP_ENABLE_DEBUG_LOG
    )
endif ()

target_link_libraries(registration_cli PRIVATE
        TIFF::TIFF
        Eigen3::Eigen
        Open3D::Open3D
)

if(MSVC)
    target_compile_definitions(registration_cli PRIVATE
            _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING
    )
endif()

set_target_properties(registration_cli PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
        "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG
        "${CMAKE_BINARY_DIR}/bin"
)

# deploy open3d runtime files
#-----------------------------------------------------------------------------------------------------------------------
# find the necessary dlls
file(GLOB_RECURSE OPEN3D_RUNTIME_DLLS
        CONFIGURE_DEPENDS
        LIST_DIRECTORIES FALSE
        "${OPEN3D_ROOT}/*.dll"
)

# verify that dlls were found
if(NOT OPEN3D_RUNTIME_DLLS)
    message(FATAL_ERROR
            "No Open3D DLLs found under ${OPEN3D_ROOT}"
    )
endif()

# print discovered DLLs
foreach(_dll IN LISTS OPEN3D_RUNTIME_DLLS)
    message(STATUS "Found Open3D DLL: ${_dll}")
endforeach()

# copy the dlls alongside the executable
foreach(_dll IN LISTS OPEN3D_RUNTIME_DLLS)
    add_custom_command(TARGET registration_cli POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${_dll}"
            "$<TARGET_FILE_DIR:registration_cli>"
            VERBATIM
    )
endforeach()

# copy Open3D resources if present.
if(EXISTS "${OPEN3D_ROOT}/bin/resources")
    add_custom_command(TARGET registration_cli POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${OPEN3D_ROOT}/bin/resources"
            "$<TARGET_FILE_DIR:registration_cli>/resources"
            VERBATIM
    )
endif()

