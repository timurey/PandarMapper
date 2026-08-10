#!/bin/bash
# STEP 1: confirm the Pi <-> Pandar physical Ethernet link.
# Run this FIRST. If it fails, it is a cabling/power/port problem, not software.
set -u
echo "=== eth0 link state ==="
ip link set eth0 up 2>/dev/null || sudo ip link set eth0 up
sleep 4
carrier=$(cat /sys/class/net/eth0/carrier 2>/dev/null)
echo "carrier=$carrier  operstate=$(cat /sys/class/net/eth0/operstate)"
ip -br link show eth0
echo
if [ "$carrier" != "1" ]; then
  echo ">>> NO LINK (carrier=$carrier). The Pi sees no link partner."
  echo ">>> Check: (a) swap in the SAME cable that worked on your laptop,"
  echo "           (b) reseat both ends, (c) confirm you're in the interface box's"
  echo "               HOST port (not the lidar-side port), (d) box is powered."
  echo ">>> If a known-good cable still gives no LEDs -> Pi eth0 port is suspect;"
  echo "    a USB3 gigabit adapter is a clean workaround."
  exit 1
fi
echo ">>> LINK UP. Speed: $(cat /sys/class/net/eth0/speed 2>/dev/null) Mb/s"
echo ">>> Next: run 02_discover_pandar.sh"
