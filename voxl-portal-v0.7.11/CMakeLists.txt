cmake_minimum_required(VERSION 3.3)

# Enable c++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -pthread -ggdb -fpermissive -DENABLE_MAVLINK_SUPPORT")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# tell the linker not to worry about missing symbols in libraries
# core Modal AI libraries often have a lot of dependencies and this prevents
# higher level projects from needing to install so many dependencies at compile time
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")

option( "Build the qrb5165 binary" OFF)

if(BUILD_QRB5165)
    add_definitions(-DBUILD_QRB5165)
endif()

# The name of the application
project(voxl-portal)
set(APP_NAME voxl-portal)

# Search for the library and if its not present then use the submodules versions
# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	set(LIB_INSTALL_DIR /usr/lib64)
	find_library(LIBMODAL_PIPE_SO NAMES libmodal_pipe.so HINTS /usr/lib64 )
	find_library(LIBMODAL_JSON_SO NAMES libmodal_json.so HINTS /usr/lib64 )
	find_library(LIBVOXL_CUTILS_SO NAMES libvoxl_cutils.so HINTS /usr/lib64 )
	find_library(LIBMONGOOSE_SO NAMES libmongoose.so HINTS /usr/lib64 )
	find_library(LIBJPEG_TURBO_SO NAMES libturbojpeg.so HINTS /usr/lib64 )

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
	message( "configure for ARMv7-A")
	set(LIB_INSTALL_DIR /usr/lib)
	find_library(LIBMODAL_PIPE_SO NAMES libmodal_pipe.so HINTS /usr/lib )
	find_library(LIBMODAL_JSON_SO NAMES libmodal_json.so HINTS /usr/lib )
	find_library(LIBVOXL_CUTILS_SO NAMES libvoxl_cutils.so HINTS /usr/lib )
	find_library(LIBMONGOOSE_SO NAMES libmongoose.so HINTS /usr/lib )
	find_library(LIBJPEG_TURBO_SO NAMES libturbojpeg.so HINTS /usr/lib )
	set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -march=armv7-a -mfloat-abi=softfp -mfpu=neon -Wno-unused-function -DENABLE_MAVLINK_SUPPORT")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -g -pthread -Wall -fPIC")
	set(CMAKE_C_FLAGS "-march=armv7-a -mfloat-abi=softfp -mfpu=neon -pthread -Wno-unused-function")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 -g -pthread -Wall -fPIC" )
else()
	set(LIB_INSTALL_DIR /usr/lib)
	find_library(LIBMODAL_PIPE_SO NAMES libmodal_pipe.so HINTS /usr/lib )
	find_library(LIBMODAL_JSON_SO NAMES libmodal_json.so HINTS /usr/lib )
	find_library(LIBVOXL_CUTILS_SO NAMES libvoxl_cutils.so HINTS /usr/lib )
	find_library(LIBMONGOOSE_SO NAMES libmongoose.so HINTS /usr/lib )
	find_library(LIBJPEG_TURBO_SO NAMES libturbojpeg.so HINTS /usr/lib )
endif()

file(GLOB_RECURSE all_src_files "src/*.c*")

include_directories(
	include/
)

# Add all the files needed to generate the executable
add_executable(${APP_NAME} ${all_src_files} )

# Link against all the libraries that are needed for the application to run
target_link_libraries( ${APP_NAME}
	${LIBMODAL_PIPE_SO}
	${LIBMODAL_JSON_SO}
	${LIBVOXL_CUTILS_SO}
	${LIBMONGOOSE_SO}
	${LIBJPEG_TURBO_SO}
)

install(
    TARGETS voxl-portal
    DESTINATION /usr/bin
)

install(
	DIRECTORY web_root
	DESTINATION /data
)
