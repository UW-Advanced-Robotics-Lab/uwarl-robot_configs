#!/usr/bin/env bash
# Install with CUDA, check with jetson_info to see if our OpenCV is CUDA enabled, otherwise you may ran out of memory
# 2019 Michael de Gans @ https://github.com/mdegans/nano_build_opencv/blob/master/build_opencv.sh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
set -e

# change default constants here:
readonly PREFIX=/usr/local  # install prefix, (can be ~/.local for a user install)
readonly DEFAULT_VERSION=4.2.0  # controls the default version (gets reset by the first argument)
readonly CPUS=$(nproc)  # controls the number of jobs

# better board detection. if it has 6 or more cpus, it probably has a ton of ram too
if [[ $CPUS -gt 5 ]]; then
    # something with a ton of ram
    JOBS=$[ $CPUS - 1 ]
    ic "Build with all cpus n:$CPUS"
else
    JOBS=1  # you can set this to 4 if you have a swap file
    # otherwise a Nano will choke towards the end of the build
fi

cleanup () {
# https://stackoverflow.com/questions/226703/how-do-i-prompt-for-yes-no-cancel-input-in-a-linux-shell-script
    while true ; do
        ic_wrn "Do you wish to remove temporary build files in ${JX_LINUX}/build_opencv ?  (yes/no)"
        if ! [[ "$1" -eq "--test-warning" ]] ; then
            ic_wrn "(Doing so may make running tests on the build later impossible)"
        fi
        read rm_old

        if [ "$rm_old" = "yes" ]; then
            ic "** Remove other OpenCV first"
            sudo apt -y purge *libopencv*
            sudo rm -rf ${JX_LINUX}/build_opencv
            break
        elif [ "$rm_old" = "no" ]; then
            break
        else
            ic_wrn "Please answer yes or no"
            exit 0
        fi
    done
}

setup () {
    ic_title "Prep (1/4)"
    cd $JX_LINUX
    if [[ -d "build_opencv" ]] ; then
        ic "It appears an existing build exists in ${JX_LINUX}/build_opencv"
        cleanup
    fi
    mkdir build_opencv
    cd build_opencv
}

git_source () {
    local version=$1
    ic_title "Getting version '$version' of OpenCV (2/4)"
    cd $JX_LINUX/build_opencv
    curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
    curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
    unzip opencv-${version}.zip
    unzip opencv_contrib-${version}.zip
    rm opencv-${version}.zip opencv_contrib-${version}.zip
    cd opencv-${version}/
}

install_dependencies () {
    # open-cv has a lot of dependencies, but most can be found in the default
    # package repository or should already be installed (eg. CUDA).
    ic_title "Installing build dependencies. (1/4)"
    sudo apt-get update
    sudo apt-get dist-upgrade -y --autoremove
    sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    sudo apt-get install -y python3.8-dev python-dev python-numpy python3-numpy
    sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
    sudo apt-get install -y libv4l-dev v4l-utils qv4l2
    sudo apt-get install -y curl
}

configure () {
    local version=$1
    local CMAKEFLAGS="
        -D BUILD_EXAMPLES=OFF
        -D BUILD_opencv_python2=ON
        -D BUILD_opencv_python3=ON
        -D CMAKE_BUILD_TYPE=RELEASE
        -D CMAKE_INSTALL_PREFIX=${PREFIX}
        -D CUDA_ARCH_BIN=5.3,6.2,7.2,8.7
        -D CUDA_ARCH_PTX=
        -D CUDA_FAST_MATH=ON
        -D CUDNN_VERSION='8.0'
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 
        -D ENABLE_NEON=ON
        -D OPENCV_DNN_CUDA=ON
        -D OPENCV_ENABLE_NONFREE=ON
        -D OPENCV_EXTRA_MODULES_PATH=${JX_LINUX}/build_opencv/opencv_contrib-${version}/modules
        -D OPENCV_GENERATE_PKGCONFIG=ON
        -D WITH_CUBLAS=ON
        -D WITH_CUDA=ON
        -D WITH_CUDNN=ON
        -D WITH_GSTREAMER=ON
        -D WITH_LIBV4L=ON
        -D WITH_OPENGL=ON"

    if [[ "$2" != "test" ]] ; then
        CMAKEFLAGS="
        ${CMAKEFLAGS}
        -D BUILD_PERF_TESTS=OFF
        -D BUILD_TESTS=OFF"
    fi

    echo "cmake flags: ${CMAKEFLAGS}"

    cd $JX_LINUX/build_opencv/opencv-${version}/
    mkdir build
    cd build
    cmake ${CMAKEFLAGS} .. 2>&1 | tee -a configure.log
}

main () {

    local VER=${DEFAULT_VERSION}

    # parse arguments
    if [[ "$#" -gt 0 ]] ; then
        VER="$1"  # override the version
    fi

    if [[ "$#" -gt 1 ]] && [[ "$2" == "test" ]] ; then
        DO_TEST=1
    fi
    # prepare for the build:
    setup
    install_dependencies
    git_source ${VER}

    if [[ ${DO_TEST} ]] ; then
        configure ${VER} test
    else
        configure ${VER}
    fi

    # start the build
    ic_title "Building ... (3/4)"
    make -j${JOBS} 2>&1 | tee -a build.log

    if [[ ${DO_TEST} ]] ; then
        make test 2>&1 | tee -a test.log
    fi

    ic_title "Install ... (4/4)"
    ic_wrn "This will take a while"
    # avoid a sudo make install (and root owned files in ~) if $PREFIX is writable
    if [[ -w ${PREFIX} ]] ; then
        make install 2>&1 | tee -a install.log
    else
        sudo make install 2>&1 | tee -a install.log
    fi
    echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.zshrc
    echo 'export PYTHONPATH=/usr/local/lib/python3.8/site-packages/:$PYTHONPATH' >> ~/.zshrc

    ic_title "OpenCV ${VER} ready to be used"
    ic "Bye :-)"
}

main "$@"