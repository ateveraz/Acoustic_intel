#!/bin/bash

# File for compilation. 
# input [1]: indicates if the compilation is for core2 or arm. 
# input [2] (optional): defines if the arm compilation will be sent to the drone. 
# input [3] (optional): last ip value for sending. 
# input [4] (optional): port for sending. 
# input [5] (optional): path on the drone for sending. 
# input [6] (optional): files to send. 

# Function to display help message
function display_help() {
    echo "Usage: $0 [TARGET] [SEND_TO_DRONE] [IP_LAST_OCTET] [PORT_IP] [DRON_PATH] [FILES2SEND]"
    echo
    echo "Arguments:"
    echo "  TARGET          Compilation target, either 'core' or 'arm'."
    echo "  SEND_TO_DRONE   Optional. 'yes' to send the compiled files to the drone."
    echo "  IP_LAST_OCTET   Optional. Last octet of the drone's IP address."
    echo "  PORT_IP         Optional. Port for sending files to the drone."
    echo "  DRON_PATH       Optional. Path on the drone where files will be sent."
    echo "  FILES2SEND      Optional. Files to send to the drone."
    echo
    echo "Example:"
    echo "  $0 arm yes 100 22 /path/to/destination file_to_send"
    exit 0
}

# Check if -h is used as an argument
if [[ "$1" == "-h" ]]; then
    display_help
fi

PROJECT_NAME=$(basename "$PWD")
PROJECT_NAME_FLAIR="Camille"

TARGET=$1
SEND_TO_DRONE=$2
IP_LAST_OCTET=$3
PORT_IP=$4
DRON_PATH=$5
FILES2SEND=$6

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

# Send to drone if specified
if [ "$SEND_TO_DRONE" == "yes" ] && [ "$TARGET" == "arm" ]; then
    if [ -z "$IP_LAST_OCTET" ]; then
        echo "IP last octet not provided."
        exit 1
    fi
    # Set the path of the compiled files
    COMPILED_FILES_PATH="$FLAIR_ROOT/flair-install/bin/demos/armv7a-neon/$PROJECT_NAME_FLAIR/$FILES2SEND"
    DRONE_IP="172.26.209.$IP_LAST_OCTET"

    scp -r $COMPILED_FILES_PATH root@$DRONE_IP:/home/root/$DRON_PATH
fi

if [ "$SEND_TO_DRONE" == "yes" ] && [ "$TARGET" == "core" ]; then
    if [ -z "$IP_LAST_OCTET" ]; then
        echo "IP last octet not provided."
        exit 1
    fi
    if [ -z "$PORT_IP" ]; then
        echo "Port not provided."
        exit 1
    fi
    # Set the path of the compiled files
    COMPILED_FILES_PATH="$FLAIR_ROOT/flair-install/bin/demos/core2/$PROJECT_NAME_FLAIR/$FILES2SEND"
    DRONE_IP="172.26.209.$IP_LAST_OCTET"

    scp -rP $PORT_IP $COMPILED_FILES_PATH root@$DRONE_IP:/home/root/$DRON_PATH
fi