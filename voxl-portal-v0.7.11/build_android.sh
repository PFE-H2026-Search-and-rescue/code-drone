# !/bin/bash

set -e

# Setup the android NDK
export NDK=~/Android/Sdk/ndk-bundle/

# Where we will be installing the android build
BUILD_INSTALL_DIR=./android_build/

####################################################################################
## Parse the command line arguments
####################################################################################
POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -b|--build_dir)
    BUILD_DIR_INPUT="$2"
    shift # past argument
    shift # past value
    ;;
    -n|--ndk)
    NDK_INPUT="$2"
    shift # past argument
    shift # past value
    ;;
    -h|--help)
    HELP_INPUT="wasSet"
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters



# Check if the help was selected and if it was then print it
if [[ -v HELP_INPUT ]]; 
then 
    echo "Build voxl-portal for Android."
    echo "This will also build all the needed dependencies."
    echo
    echo "Usage: ./build_android.sh -n <ndk_install_location> -b <output_build_dir>"
    echo "Options:"
    echo "      -n,--ndk        : The location that the NDK is installed."
    echo "                        Default is \"~/Android/Sdk/ndk-bundle/\""
    echo "                        The argument is optional"
    echo "      -b,--build_dir  : The location to put the build output"
    echo "                        Default is \"./android_build/\""
    echo "                        The argument is optional"
    echo "      -h,--help       : Display this usage help"
    exit
fi


# Check if the build dir is set and take appropriate action
if [ -z ${BUILD_DIR_INPUT+x} ]; 
then 
	# Make it absolute
	CWD=$(pwd)
	BUILD_INSTALL_DIR=${CWD}/${BUILD_INSTALL_DIR}
	echo "Build dir is not set. Using ${BUILD_INSTALL_DIR}"
else 
	echo "Build directory was manually set to ${BUILD_DIR_INPUT}"
	BUILD_INSTALL_DIR=${BUILD_DIR_INPUT}
fi

# Check if the NDK is set and take appropriate action
if [ -z ${NDK_INPUT+x} ]; 
then 
	echo "NDK is not set. Using ${NDK}"
else 
	echo "NDK was manually set to ${NDK_INPUT}"
	export NDK=${NDK_INPUT}
fi

####################################################################################
## Set the output dirs
####################################################################################

# Where the output files will go
LIB_DIR=${BUILD_INSTALL_DIR}/lib/
INCLUDE_DIR=${BUILD_INSTALL_DIR}/include/
BIN_DIR=${BUILD_INSTALL_DIR}/bin/

# Make all the needed output directories
mkdir -p ${LIB_DIR}
mkdir -p ${INCLUDE_DIR}
mkdir -p ${BIN_DIR}

####################################################################################
## Build the libturbojpeg
####################################################################################
if [ -f "${LIB_DIR}/libturbojpeg.so" ]; then
    echo "${LIB_DIR}/libturbojpeg.so exists, so skipping build"
else
    cd libjpeg-turbo
    rm -rf build/
    mkdir -p build
    cd build
    NDK_PATH=$NDK
    TOOLCHAIN=clang
    ANDROID_VERSION=21
    mkdir build
    cmake -G"Unix Makefiles" -DANDROID_ABI=arm64-v8a -DANDROID_ARM_MODE=arm -DANDROID_PLATFORM=android-${ANDROID_VERSION} -DANDROID_TOOLCHAIN=${TOOLCHAIN}   -DCMAKE_ASM_FLAGS="--target=aarch64-linux-android${ANDROID_VERSION}" -DCMAKE_TOOLCHAIN_FILE=${NDK_PATH}/build/cmake/android.toolchain.cmake -DCMAKE_INSTALL_LIBDIR=../../../android_build/lib -DCMAKE_INSTALL_INCLUDEDIR=../../../android_build/include ../
    make install
    cp ./libturbojpeg.so ${LIB_DIR}
    cd ../../
fi

####################################################################################
## Build the Application
####################################################################################

cp ${NDK}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so ${LIB_DIR}

# The directory where we build android from
ANDROID_BUILD_DIR=./android/

# The location of the lib and include 
VOXL_PORTAL_EXE=${ANDROID_BUILD_DIR}/libs/arm64-v8a/voxl-portal
INCLUDE_BUILD_DIR=${ANDROID_BUILD_DIR}../library/include/

# Clean the Android build artifacts
rm -rf ${ANDROID_BUILD_DIR}/obj
rm -rf ${ANDROID_BUILD_DIR}/libs
echo $(pwd)
# Make the android libs
CWD=$(pwd)
cd ${ANDROID_BUILD_DIR}/jni
$NDK/ndk-build -e ALL_BUILD_DIR=${BUILD_INSTALL_DIR} -e APP_ABI=arm64-v8a
cd ${CWD}


####################################################################################
## Install the library
####################################################################################

# Move the files to the Install directory 
cp ${VOXL_PORTAL_EXE} ${BIN_DIR}
cp -r ./web_root ${BUILD_INSTALL_DIR}/



