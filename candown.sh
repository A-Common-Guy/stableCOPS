#!/bin/sh
# Take down one or more CAN interfaces. Default: can0.
#   sudo ./candown.sh            # can0 only
#   sudo ./candown.sh can0 can1  # both legs
[ $# -eq 0 ] && set -- can0
for iface in "$@"; do
    ip link set down "$iface"
done
