#!/bin/bash
# Installed as a root-owned executable on vehicles that use host clock sync.
set -euo pipefail

if [[ $# == 1 && $1 == --probe ]]; then
  exit 0
fi
if [[ $# != 1 || ! $1 =~ ^[1-9][0-9]{9}(\.[0-9]{3})?$ ]]; then
  echo 'Expected one UTC Unix timestamp from the controller.' >&2
  exit 2
fi
seconds=${1%%.*}
if (( seconds < 1704067200 || seconds >= 4102444800 )); then
  echo 'The controller clock is outside the supported date range.' >&2
  exit 2
fi

active='[r]os2[[:space:]]+launch|[w]heeltec_robot_node|[c]ontroller_server|[s]lam_toolbox_node|[f]rontier_tasks.py|[t]eleop_twist_keyboard|[d]emo/server[.]py.*--backend[ =]+ros([[:space:]]|$)'
if /usr/bin/pgrep -f -- "$active" >/dev/null; then
  echo 'Stop vehicle ROS services before changing the system clock.' >&2
  exit 3
fi

/usr/bin/date --utc --set="@$1" '+%s.%N'
if [[ -c /dev/rtc0 ]] && ! /usr/sbin/hwclock --systohc --utc --rtc=/dev/rtc0; then
  echo 'The system clock was set. RTC write failed.' >&2
fi
