#!/bin/bash

# File for compilation. 
# input [1]: indicates if the compilation is for core2 or arm. 
# input [2] (optional): defines if the arm compilation will be sent to the drone. 
# input [3] (optional): last ip value for sending. 

# Get the project name from the current working directory
cd .. 

PROJECT_NAME=$(basename "$PWD")
PROJECT_NAME_FLAIR="Camille"

TARGET=$1
SEND_TO_DRONE=$2
IP_LAST_OCTET=$3

# Set the build directory based on the target
if [ "$TARGET" == "core" ]; then
    BUILD_DIR="$FLAIR_ROOT/flair-build/mydemos/$PROJECT_NAME/build"
elif [ "$TARGET" == "arm" ]; then
    BUILD_DIR="$FLAIR_ROOT/flair-build/mydemos/$PROJECT_NAME/build_armv7a_neon"
else
    echo "Invalid target specified. Use 'core' or 'arm'."
    exit 1
fi

# Compile the project
cd $BUILD_DIR
make clean
make install -j`nproc`

# Set the path of the compiled files
COMPILED_FILES_PATH="$FLAIR_ROOT/flair-install/bin/demos/armv7a-neon/$PROJECT_NAME_FLAIR"

# Send to drone if specified
if [ "$SEND_TO_DRONE" == "yes" ] && [ "$TARGET" == "arm" ]; then
    if [ -z "$IP_LAST_OCTET" ]; then
        echo "IP last octet not provided."
        exit 1
    fi
    DRONE_IP="172.26.209.$IP_LAST_OCTET"
    scp -r $COMPILED_FILES_PATH/* root@$DRONE_IP:/demos/CamilleYaw/
fi