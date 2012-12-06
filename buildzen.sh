#!/bin/bash

if [[ ! $# -ge 2 ]] ; then
	echo "Usage: buildzen.sh {GCC Path} {Version Code} {CleanFirst}"
	echo "e.g: ./buildzen.sh /opt/ndk/bin/arm-linux-gnueabi- 0 true"
	echo "Use version 0 if not an official build!"
	exit 1
fi

if [[ ! ${2} =~ ^[0-9]+$ ]] ; then
	echo "ERROR: Version must be an integer"
	exit 1
fi

if [[ ! ${2} -eq 0 ]] ; then
        echo -n "Are you bbedward? (Y/N): "
        read answer
        answer=`echo $answer | tr '[A-Z]' '[a-z]'`
        if [[ $answer != 'y' ]] ; then
                echo "Use version code 0 not ${2}!"
                exit 1;
        fi
fi

cleanFirst=${3}
cleanFirst=`echo $cleanFirst | tr '[A-Z]' '[a-z]'`

if [[ $cleanFirst == "true" ]]; then
	ARCH=arm CROSS_COMPILE=${1} make clean
fi

if [[ ! -f .config ]] ; then
	ARCH=arm CROSS_COMPILE=${1} make tuna_zen_defconfig
fi

sed -i s/CONFIG_ZEN_INFO_VERSION_CODE=.*/CONFIG_ZEN_INFO_VERSION_CODE=${2}/ .config

ARCH=arm CROSS_COMPILE=${1} make -j2

if [[ $? -ne 0 ]] ; then
	echo "Kernel build failed"
	exit 1
fi

cp arch/arm/boot/zImage zip_template/kernel/
rm -fr zip_template/system/modules/*
find . -name "*\.ko" -exec cp {} zip_template/system/modules/ \;

cd zip_template
zip -r9 zen_unofficial_${2}.zip *
cd ..
echo "zip_template/zen_unofficial_${2}.zip created"
