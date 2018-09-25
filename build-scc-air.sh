KERNEL_DIR=.
INSTALL_MOD_PATH=$PWD/../../board/swarco/ccm2200/rootfs_overlay
MOD_DIRECTORY=2.6.21.7-weiss-ccm2200
#OUTPUT_DIR=$PWD/../output-${PWD##*linux-}
OUTPUT_DIR=../output-sccair
IMAGE_DIR=../../tftp_root/


test -d $OUTPUT_DIR || mkdir -p $OUTPUT_DIR
test -d $INSTALL_MOD_PATH || mkdir -p $INSTALL_MOD_PATH

if [ -f /usr/bin/kmod ]; then
# Fedora 17 comes with depmod from kmod 7 package instead of modules_init_tools, which needs this files:

    test -d $INSTALL_MOD_PATH/lib/modules/$MOD_DIRECTORY || mkdir -p $INSTALL_MOD_PATH/lib/modules/$MOD_DIRECTORY

    touch $INSTALL_MOD_PATH/lib/modules/$MOD_DIRECTORY/modules.order
    touch $INSTALL_MOD_PATH/lib/modules/$MOD_DIRECTORY/modules.builtin
fi

CPU=arm
TOOLCHAIN=$PWD/../../output/host/
PATH=$PATH:$TOOLCHAIN/usr/bin
export PATH

# add path for u-boot mkimage tool
PATH=$PATH:$PWD/../../u-boot/u-boot-v2010.09-ccm2200/tools
export PATH

if [ -z "$1" ]; then
    set zImage
    #uImage  modules modules_install
fi


# KBUILD_VERBOSE=1
make -C "$KERNEL_DIR" ARCH=arm CROSS_COMPILE="arm-buildroot-linux-gnueabi-"       \
                      INSTALL_MOD_PATH=$INSTALL_MOD_PATH        \
                      O=$OUTPUT_DIR $@ || exit $?


# if grep "CONFIG_UBIFS_FS=y" $OUTPUT_DIR/.config >/dev/null 2>&1
# then
#     cp $OUTPUT_DIR/arch/arm/boot/uImage $IMAGE_DIR/uImage-ccm2200-ubifs.bin
# else
#     cp $OUTPUT_DIR/arch/arm/boot/uImage $IMAGE_DIR/uImage-ccm2200dev.bin
# fi


# Local Variables:
# mode: shell-script
# compile-command: "./build-ccm2200.sh"
# End:
