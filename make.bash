#!/bin/bash

SOURCE_FILE=${1:-main.cpp}

BUILD_DIR="build"


rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

SOURCE_FILE_PATH="../$SOURCE_FILE"

cmake .. -D SOURCE_FILE=$SOURCE_FILE_PATH
make

./mujoco ../xml/two_link.xml