cmake_minimum_required(VERSION 3.24)

# Configure vcpkg before project() loads the toolchain.
if(NOT VCPKG_ROOT AND DEFINED ENV{VCPKG_ROOT})
    set(VCPKG_ROOT "$ENV{VCPKG_ROOT}")
endif()

if(NOT VCPKG_ROOT)
    message(FATAL_ERROR "Please specify -DVCPKG_ROOT=/path/to/vcpkg")
endif()

file(TO_CMAKE_PATH "${VCPKG_ROOT}" VCPKG_ROOT)
set(_VCPKG_TOOLCHAIN "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

if(NOT EXISTS "${_VCPKG_TOOLCHAIN}")
    message(FATAL_ERROR "Cannot find vcpkg toolchain: ${_VCPKG_TOOLCHAIN}")
endif()

# Keep the Windows default. vcpkg selects x64-linux for native x86-64 Linux.
if(CMAKE_HOST_WIN32 AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "x64-windows")
endif()

set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_TOOLCHAIN}" CACHE FILEPATH "vcpkg toolchain")

project(registration_cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Each prebuilt Open3D archive contains a single build configuration.
get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(IS_MULTI_CONFIG)
    message(FATAL_ERROR
            "Use a single-config generator such as Ninja. On Linux, configure "
            "with -DCMAKE_BUILD_TYPE=Release."
    )
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _open3d_processor)
    if(NOT _open3d_processor MATCHES "^(x86_64|amd64)$" OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR
                "This Linux Open3D setup requires a 64-bit x86-64 target. "
                "Detected: ${CMAKE_SYSTEM_PROCESSOR}."
        )
    endif()
    set(_default_build_type "Release")
elseif(WIN32)
    set(_default_build_type "Debug")
else()
    message(FATAL_ERROR "This Open3D setup supports Windows x64 and Linux x86-64.")
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE "${_default_build_type}" CACHE STRING "Build configuration" FORCE)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" _open3d_build_type)
set(OPEN3D_VERSION "0.20.0")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(NOT _open3d_build_type STREQUAL "RELEASE")
        message(FATAL_ERROR
                "The Linux build requires -DCMAKE_BUILD_TYPE=Release. "
                "Received '${CMAKE_BUILD_TYPE}'."
        )
    endif()
    set(OPEN3D_ARCHIVE_NAME "open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VERSION}.tar.xz")
    set(OPEN3D_SHA256 "fc34e846419afa21aa96eb3c61a62e967e3d64d6d9ad5a8d27d5c56046e11ccf")
    set(OPEN3D_PACKAGE_SUFFIX "linux-release")
elseif(_open3d_build_type STREQUAL "DEBUG")
    set(OPEN3D_ARCHIVE_NAME "open3d-devel-windows-amd64-${OPEN3D_VERSION}-dbg.zip")
    set(OPEN3D_SHA256 "e4c5e04669f80c9464f2fcab61cea78ab9c18080e94091b7fcb24c53b8c87801")
    set(OPEN3D_PACKAGE_SUFFIX "windows-debug")
elseif(_open3d_build_type STREQUAL "RELEASE" OR _open3d_build_type STREQUAL "RELWITHDEBINFO")
    set(OPEN3D_ARCHIVE_NAME "open3d-devel-windows-amd64-${OPEN3D_VERSION}.zip")
    set(OPEN3D_SHA256 "ae2843b64da4d82e0cabf82462a9fda12921ea5be9a9d593f14a7a21a44a97fe")
    set(OPEN3D_PACKAGE_SUFFIX "windows-release")
else()
    message(FATAL_ERROR "Unsupported Windows build configuration: ${CMAKE_BUILD_TYPE}")
endif()

set(OPEN3D_URL "https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/${OPEN3D_ARCHIVE_NAME}")
set(OPEN3D_DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/_deps/downloads")
set(OPEN3D_ARCHIVE "${OPEN3D_DOWNLOAD_DIR}/${OPEN3D_ARCHIVE_NAME}")
set(OPEN3D_ROOT "${CMAKE_BINARY_DIR}/_deps/open3d-${OPEN3D_VERSION}-${OPEN3D_PACKAGE_SUFFIX}")
set(_open3d_extracted_stamp "${OPEN3D_ROOT}/.extracted-${OPEN3D_SHA256}")

# EXPECTED_HASH also checks a cached archive and downloads it again if the hash differs.
file(MAKE_DIRECTORY "${OPEN3D_DOWNLOAD_DIR}")
file(DOWNLOAD
        "${OPEN3D_URL}" "${OPEN3D_ARCHIVE}"
        EXPECTED_HASH "SHA256=${OPEN3D_SHA256}"
        TLS_VERIFY ON
        INACTIVITY_TIMEOUT 60
        SHOW_PROGRESS
        STATUS _open3d_download_status
)
list(GET _open3d_download_status 0 _open3d_download_code)
if(NOT _open3d_download_code EQUAL 0)
    list(GET _open3d_download_status 1 _open3d_download_message)
    message(FATAL_ERROR "Open3D download failed: ${_open3d_download_message}")
endif()

# Discover the package config even when the archive contains an outer directory.
file(GLOB_RECURSE OPEN3D_CONFIG_FILES LIST_DIRECTORIES FALSE "${OPEN3D_ROOT}/Open3DConfig.cmake")
if(NOT EXISTS "${_open3d_extracted_stamp}" OR NOT OPEN3D_CONFIG_FILES)
    message(STATUS "Extracting ${OPEN3D_ARCHIVE_NAME}")
    file(REMOVE_RECURSE "${OPEN3D_ROOT}")
    file(MAKE_DIRECTORY "${OPEN3D_ROOT}")
    file(ARCHIVE_EXTRACT INPUT "${OPEN3D_ARCHIVE}" DESTINATION "${OPEN3D_ROOT}")
    file(GLOB_RECURSE OPEN3D_CONFIG_FILES LIST_DIRECTORIES FALSE "${OPEN3D_ROOT}/Open3DConfig.cmake")
endif()

list(LENGTH OPEN3D_CONFIG_FILES _open3d_config_count)
if(NOT _open3d_config_count EQUAL 1)
    message(FATAL_ERROR
            "Expected one Open3DConfig.cmake under ${OPEN3D_ROOT}, "
            "found ${_open3d_config_count}: ${OPEN3D_CONFIG_FILES}"
    )
endif()

list(GET OPEN3D_CONFIG_FILES 0 _open3d_config_file)
get_filename_component(_open3d_config_dir "${_open3d_config_file}" DIRECTORY)
set(Open3D_DIR "${_open3d_config_dir}" CACHE PATH "Open3D configuration directory" FORCE)
find_package(Open3D "${OPEN3D_VERSION}" EXACT CONFIG REQUIRED NO_DEFAULT_PATH)

if(NOT EXISTS "${_open3d_extracted_stamp}")
    file(TOUCH "${_open3d_extracted_stamp}")
endif()

find_package(TIFF REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

add_executable(registration_cli ${SOURCES} ${HEADERS})
target_include_directories(registration_cli PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

option(ENABLE_DEBUG_LOG "Enable debug logging" ON)
if(ENABLE_DEBUG_LOG)
    target_compile_definitions(registration_cli PRIVATE IRTOOLS_IP_ENABLE_DEBUG_LOG)
endif()

target_link_libraries(registration_cli PRIVATE
        TIFF::TIFF
        Eigen3::Eigen
        Open3D::Open3D
)

if(MSVC)
    target_compile_definitions(registration_cli PRIVATE _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING)
endif()

set_target_properties(registration_cli PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin"
)

# On Linux, CMake adds the imported shared libraries' directory to the build RPATH.
if(WIN32)
    file(GLOB_RECURSE OPEN3D_RUNTIME_DLLS
            CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE "${OPEN3D_ROOT}/*.dll"
    )
    if(NOT OPEN3D_RUNTIME_DLLS)
        message(FATAL_ERROR "No Open3D DLLs found under ${OPEN3D_ROOT}")
    endif()

    foreach(_dll IN LISTS OPEN3D_RUNTIME_DLLS)
        add_custom_command(TARGET registration_cli POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${_dll}" "$<TARGET_FILE_DIR:registration_cli>"
                VERBATIM
        )
    endforeach()

    # Locate resources next to Open3D.dll, including in nested archives.
    foreach(_dll IN LISTS OPEN3D_RUNTIME_DLLS)
        get_filename_component(_dll_name "${_dll}" NAME)
        if(_dll_name STREQUAL "Open3D.dll")
            get_filename_component(_open3d_bin_dir "${_dll}" DIRECTORY)
            if(IS_DIRECTORY "${_open3d_bin_dir}/resources")
                add_custom_command(TARGET registration_cli POST_BUILD
                        COMMAND "${CMAKE_COMMAND}" -E copy_directory
                        "${_open3d_bin_dir}/resources" "$<TARGET_FILE_DIR:registration_cli>/resources"
                        VERBATIM
                )
            endif()
            break()
        endif()
    endforeach()
endif()