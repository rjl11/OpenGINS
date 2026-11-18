#!/bin/sh
# Build inslib_replay and replay one field log in both height modes
# (--height gnss and --height baro, in parallel). Run from the repo root on
# a host with gcc (not the Pico toolchain). Extra arguments go to both runs.
#
#   firmware/tools/inslib_replay.sh LOG.bin [inslib_replay options]
#
# Writes temp/replay/<log name>_gnss.csv and _baro.csv, with INSLIB's
# warnings in the matching .txt. Plot them with
#   python firmware/tools/inslib_replay_plot.py OUT.png \
#       temp/replay/<log name>_gnss.csv=gnss temp/replay/<log name>_baro.csv=baro
set -e
[ $# -ge 1 ] || { echo "usage: $0 LOG.bin [inslib_replay options]" >&2; exit 2; }
log=$1; shift
I=external/INSLIB
mkdir -p temp/replay
gcc -O2 -std=gnu11 -Wall -DKALMAN_MAX_STATE_SIZE=18 -DKALMAN_MAX_NOISE_SIZE=24 \
    -I$I/src -I$I/KFCore/c -I firmware/app firmware/tools/inslib_replay.c \
    $I/src/ins.c $I/src/ahrs.c $I/src/baro_alt.c $I/src/nav_suite.c $I/src/geodetic_toolbox.c \
    $I/src/magnetic_model.c $I/src/log.c $I/KFCore/c/linalg.c $I/KFCore/c/kalman_udu.c \
    $I/KFCore/c/miniblas.c -lm -o temp/inslib_replay
name=$(basename "$log" .bin)
for m in gnss baro; do
    temp/inslib_replay "$log" "temp/replay/${name}_$m.csv" --height $m "$@" \
        > "temp/replay/${name}_$m.txt" 2>&1 &
done
wait
for m in gnss baro; do
    echo "== $m"; grep -v "disagreement" "temp/replay/${name}_$m.txt" | tail -5
done
