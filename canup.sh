#!/bin/sh
# Bring up one or more CAN interfaces at 1 Mbit/s. Default: can0.
#   sudo ./canup.sh              # can0 only
#   sudo ./canup.sh can0 can1    # both legs (two_leg_demo)
[ $# -eq 0 ] && set -- can0
for iface in "$@"; do
    ip link set "$iface" type can bitrate 1000000
    # Multi-drive commissioning can burst several PDOs per SYNC. A larger TX queue
    # absorbs short scheduling bursts; if it still overflows, slow the SYNC period or
    # reduce the mapped PDO traffic.
    ip link set "$iface" txqueuelen 1000
    ip link set up "$iface"
done
