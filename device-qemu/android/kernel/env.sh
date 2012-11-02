#!/bin/sh

if [ $# -ne 1 ]
then
	echo "Usage: ${0} android-platform-path"
	exit 1
fi

ARCH=arm
CROSS_COMPILE=${1}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-
export ARCH
export CROSS_COMPILE
