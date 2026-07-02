#!/bin/bash
# Run ikalibr_prog headless on a given config, wait for the solve to finish, then
# stop it (ikalibr_prog blocks on the Pangolin viewer after solving is done).
# Usage: run_ikalibr.sh <config_path>
CONFIG="$1"
LOG="${2:-/tmp/ikalibr_solve.log}"

# set ROS env BEFORE sourcing: ROS's profile.d/10.roslaunch.sh references
# ROS_MASTER_URI, which trips 'set -u' style setups if unset. (Do not use 'set -u' here.)
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
source /home/iKalibr/install/setup.bash

roscore >/tmp/roscore.log 2>&1 &
sleep 5

xvfb-run -a -s "-screen 0 1280x720x24" \
  rosrun ikalibr ikalibr_prog _config_path:="$CONFIG" >"$LOG" 2>&1 &
PID=$!

status=timeout
for _ in $(seq 1 2400); do   # up to ~80 min guard
  if grep -q "solving and outputting finished" "$LOG"; then status=ok; break; fi
  if grep -qiE "outdated or broken|terminate called|Segmentation fault|what\(\):|\[ERROR\]|core dumped" "$LOG"; then status=error; break; fi
  if ! kill -0 "$PID" 2>/dev/null; then status=exited; break; fi
  sleep 2
done

sleep 3   # let final outputs flush
pkill -f ikalibr_prog 2>/dev/null || true
pkill -x Xvfb        2>/dev/null || true
pkill roscore        2>/dev/null || true
pkill rosmaster      2>/dev/null || true

echo "RUN_STATUS=$status"
[ "$status" = "ok" ] && exit 0 || exit 1
