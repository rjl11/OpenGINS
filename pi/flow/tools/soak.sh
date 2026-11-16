#!/bin/sh
# Soak test: run the flow program for a while and log the machine once a
# minute (CPU, RAM, temperature, throttling) plus the flow's own 5 s stats.
#
#   DUR=3600 INT=60 OUT=~/soak pi/flow/tools/soak.sh pi/flow/build/flow \
#       --dry-run --frames FRAMES --loop --realtime --fps 100
#
# Everything after the script name is the flow command. Output in $OUT:
#   log.csv   one row per $INT s (cpu_all_pct: all cores together, 100 = all
#             busy; flow_cpu_pct: the flow process, 100 = one whole core)
#   flow.log  the flow program's stderr (its 5 s stats lines)
#   done      written at the end, with the exit reason
# Run it under nohup; each row is synced to disk so a crash keeps the log.
set -u
export LC_ALL=C  # decimal points in the CSV whatever the system locale

DUR=${DUR:-3600}
INT=${INT:-60}
OUT=${OUT:-soak-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"
HZ=$(getconf CLK_TCK)
NCPU=$(nproc)

cpu_jiffies() { awk '/^cpu /{print $2+$3+$4+$5+$6+$7+$8+$9, $5+$6}' /proc/stat; }  # total idle
proc_jiffies() { awk '{print $14+$15}' "/proc/$1/stat" 2>/dev/null || echo 0; }
temp_c() {
    if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp
    else echo ""; fi
}
throttled() { command -v vcgencmd >/dev/null 2>&1 && vcgencmd get_throttled | cut -d= -f2 || echo ""; }

"$@" >/dev/null 2>"$OUT/flow.log" &
PID=$!
echo "pid $PID: $*" >"$OUT/cmd"

echo "time,elapsed_s,cpu_all_pct,flow_cpu_pct,flow_rss_mb,load1,mem_avail_mb,swap_used_mb,temp_c,throttled" >"$OUT/log.csv"
set -- $(cpu_jiffies); T0=$1; I0=$2
P0=$(proc_jiffies $PID)
START=$(date +%s)
REASON="duration reached"
while :; do
    sleep "$INT"
    if ! kill -0 $PID 2>/dev/null; then REASON="flow exited"; break; fi
    set -- $(cpu_jiffies); T1=$1; I1=$2
    P1=$(proc_jiffies $PID)
    NOW=$(date +%s)
    cpu_all=$(awk -v t=$((T1 - T0)) -v i=$((I1 - I0)) 'BEGIN{printf "%.1f", t ? 100*(t-i)/t : 0}')
    flow_cpu=$(awk -v p=$((P1 - P0)) -v hz=$HZ -v s=$INT 'BEGIN{printf "%.1f", 100*p/(hz*s)}')
    rss=$(awk '/VmRSS/{printf "%.1f", $2/1024}' /proc/$PID/status)
    load1=$(cut -d' ' -f1 /proc/loadavg)
    mem=$(awk '/MemAvailable/{printf "%.0f", $2/1024}' /proc/meminfo)
    swap=$(awk '/SwapTotal/{t=$2} /SwapFree/{f=$2} END{printf "%.0f", (t-f)/1024}' /proc/meminfo)
    echo "$(date +%H:%M:%S),$((NOW - START)),$cpu_all,$flow_cpu,$rss,$load1,$mem,$swap,$(temp_c),$(throttled)" >>"$OUT/log.csv"
    sync "$OUT/log.csv" 2>/dev/null || sync
    T0=$T1; I0=$I1; P0=$P1
    if [ $((NOW - START)) -ge "$DUR" ]; then break; fi
done

kill -TERM $PID 2>/dev/null
wait $PID 2>/dev/null
echo "$REASON at $(date '+%F %T'), $NCPU cores" >"$OUT/done"
