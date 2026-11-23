#!/bin/bash
# Run once from the vehicle administrator's terminal.
set -euo pipefail

if (( EUID != 0 )); then
  echo 'Run this installer with sudo.' >&2
  exit 1
fi
if [[ ! ${SUDO_USER:-} =~ ^[a-z_][a-z0-9_-]*$ ]]; then
  echo 'Run sudo from the vehicle account that will receive the host time.' >&2
  exit 1
fi

directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
rule=$(mktemp)
trap 'rm -f -- "$rule"' EXIT
printf '%s ALL=(root) NOPASSWD: /usr/local/sbin/r550-host-clock\n' "$SUDO_USER" > "$rule"
chmod 0440 "$rule"
/usr/sbin/visudo -cf "$rule"

install -o root -g root -m 0755 "$directory/r550_host_clock.sh" /usr/local/sbin/r550-host-clock
install -o root -g root -m 0440 "$rule" /etc/sudoers.d/r550-host-clock
echo 'R550 host clock permission installed.'
