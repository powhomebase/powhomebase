#!/bin/bash

device=$1
defines=$2

echo "Making lib - $device"

make -f scripts/ci/build/Makefile clean CONFIG_DEVICE="$device" BUILD_TEST=y
if [ $? -ne 0 ]
then
    exit 1
fi

make -f scripts/ci/build/Makefile -j16 CONFIG_DEVICE="$device" BUILD_TEST=y BUILD_DEFINES="$defines"
if [ $? -ne 0 ]
then
    exit 1
fi
