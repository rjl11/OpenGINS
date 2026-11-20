#!/bin/sh
# OV9281 on a Radxa ROCK 4SE (RK3399, Armbian / DietPi kernel). See README.md here.
#
#   sh setup.sh install    build rkisp1 + patched ov9282 for the running kernel,
#                          install them and the overlay (reboot afterwards)
#   sh setup.sh pipeline [W H]   route sensor -> CSI -> ISP (bypass) -> mainpath as
#                          8-bit raw at W x H (default 640 400); needed after every boot
#   sh setup.sh status     what is loaded, the media graph formats, a 100-frame rate test
set -eu

KVER=$(uname -r)
KBASE=$(echo "$KVER" | sed 's/-.*//')            # 6.18.60-current-rockchip64 -> 6.18.60
GIT="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain"
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${WORK:-$HOME/rkisp1-mod}

media_dev() {
    for m in /dev/media*; do
        sudo media-ctl -d "$m" -p 2>/dev/null | grep -q '^driver *rkisp1' && { echo "$m"; return; }
    done
    echo "no rkisp1 media device" >&2; exit 1
}

case "${1:-}" in
install)
    sudo apt-get install -y "linux-headers-$(echo "$KVER" | sed 's/^[^-]*-//')" \
        device-tree-compiler v4l-utils i2c-tools curl
    mkdir -p "$WORK/rkisp1" "$WORK/ov9282"
    # rkisp1: not enabled in the Armbian rockchip64 kernels, built unchanged.
    cd "$WORK/rkisp1"
    for f in Makefile rkisp1-capture.c rkisp1-common.c rkisp1-common.h rkisp1-csi.c \
             rkisp1-csi.h rkisp1-debug.c rkisp1-dev.c rkisp1-isp.c rkisp1-params.c \
             rkisp1-regs.h rkisp1-resizer.c rkisp1-stats.c; do
        curl -sfL -o "$f" "$GIT/drivers/media/platform/rockchip/rkisp1/$f?h=v$KBASE"
    done
    make -C "/lib/modules/$KVER/build" M="$PWD" CONFIG_VIDEO_ROCKCHIP_ISP1=m modules
    # ov9282: rkisp1 takes only Bayer or YUV in, so the mono Y8/Y10 codes are
    # reported as RGGB. Same MIPI data type (RAW8/RAW10), the pixels are unchanged.
    cd "$WORK/ov9282"
    curl -sfL -o ov9282.c "$GIT/drivers/media/i2c/ov9282.c?h=v$KBASE"
    sed -i 's/MEDIA_BUS_FMT_Y10_1X10/MEDIA_BUS_FMT_SRGGB10_1X10/g; s/MEDIA_BUS_FMT_Y8_1X8/MEDIA_BUS_FMT_SRGGB8_1X8/g' ov9282.c
    echo 'obj-m := ov9282.o' > Makefile
    make -C "/lib/modules/$KVER/build" M="$PWD" modules
    sudo mkdir -p "/lib/modules/$KVER/updates" /boot/overlay-user
    sudo cp "$WORK/rkisp1/rockchip-isp1.ko" "$WORK/ov9282/ov9282.ko" "/lib/modules/$KVER/updates/"
    sudo depmod -a
    dtc -@ -I dts -O dtb -o "$WORK/rk3399-rock-4se-ov9281.dtbo" "$HERE/rk3399-rock-4se-ov9281.dts"
    sudo cp "$WORK/rk3399-rock-4se-ov9281.dtbo" /boot/overlay-user/
    ENV=/boot/dietpiEnv.txt; [ -f "$ENV" ] || ENV=/boot/armbianEnv.txt
    if grep -q '^user_overlays=' "$ENV"; then
        grep -q 'rk3399-rock-4se-ov9281' "$ENV" || \
            sudo sed -i '/^user_overlays=/{s/$/ rk3399-rock-4se-ov9281/; s/= */=/}' "$ENV"
    else
        echo 'user_overlays=rk3399-rock-4se-ov9281' | sudo tee -a "$ENV" > /dev/null
    fi
    sudo usermod -aG video "$(id -un)"
    echo "Installed for $KVER. Reboot, then: sh $0 pipeline"
    echo "A kernel upgrade needs this install run again."
    ;;
pipeline)
    W=${2:-640}; H=${3:-400}
    M=$(media_dev); F="SRGGB8_1X8/${W}x$H"
    mc() { sudo media-ctl -d "$M" "$@"; }
    # The selfpath would fail link validation against a raw ISP output.
    mc -l '"rkisp1_isp":2 -> "rkisp1_resizer_selfpath":0 [0]'
    mc -V "\"ov9281 4-0060\":0 [fmt:$F]"
    mc -V "\"rkisp1_csi\":0 [fmt:$F]"
    mc -V "\"rkisp1_isp\":0 [fmt:$F crop:(0,0)/${W}x$H]"
    mc -V "\"rkisp1_isp\":2 [fmt:$F]"
    mc -V "\"rkisp1_resizer_mainpath\":0 [fmt:$F]"
    mc -V "\"rkisp1_resizer_mainpath\":1 [fmt:$F]"
    V=$(mc -e rkisp1_mainpath)
    sudo v4l2-ctl -d "$V" --set-fmt-video="width=$W,height=$H,pixelformat=RGGB"
    echo "Raw 8-bit ${W}x$H on $V; sensor controls on $(mc -e 'ov9281 4-0060')"
    ;;
status)
    lsmod | grep -E '^(rockchip_isp1|ov9282|phy_rockchip_dphy_rx0) ' || true
    M=$(media_dev)
    for p in '"ov9281 4-0060":0' '"rkisp1_isp":0' '"rkisp1_isp":2' '"rkisp1_resizer_mainpath":1'; do
        echo "$p $(sudo media-ctl -d "$M" --get-v4l2 "$p" | head -1 | tr -s '\t ' ' ')"
    done
    S=$(sudo media-ctl -d "$M" -e 'ov9281 4-0060'); V=$(sudo media-ctl -d "$M" -e rkisp1_mainpath)
    sudo v4l2-ctl -d "$S" -C exposure,analogue_gain,vertical_blanking
    sudo v4l2-ctl -d "$V" --stream-mmap=4 --stream-count=100 --stream-to=/dev/null 2>&1 \
        | grep -oE '[0-9.]+ fps' | tail -1
    ;;
*)
    sed -n '2,10p' "$0"; exit 1
    ;;
esac
