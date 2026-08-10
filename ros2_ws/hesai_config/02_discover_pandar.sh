#!/bin/bash
# STEP 2: learn the Pandar's IP + the UDP destination it streams to,
# WITHOUT yet assigning the Pi a fixed IP. Pandar blasts UDP 2368 regardless of
# whether the Pi has a matching address, so we can sniff it raw.
#
# Needs the link UP (run 01_eth0_link_check.sh first).
set -u
IFACE=eth0
echo "=== Sniffing UDP 2368 on $IFACE for 6 s (need sudo) ==="
echo "    Looking for: source IP (the lidar) and dest IP (where it streams)."
sudo timeout 6 tcpdump -ni "$IFACE" -c 8 'udp port 2368' 2>/dev/null \
  | sed -nE 's/.* IP ([0-9.]+)\.[0-9]+ > ([0-9.]+)\.2368.*/  lidar=\1  dest=\2/p' \
  | sort -u
echo
echo "If you saw a 'lidar=...' line: that is the Pandar's current IP/subnet."
echo "  - Default factory IP is 192.168.1.201 streaming to 192.168.1.100."
echo "  - This Pi's eth0 should be static at 192.168.3.100/24 (see"
echo "    system/network/60-hesai-eth0.yaml). Target setup: Pandar IP 192.168.3.201,"
echo "    dest 192.168.3.100."
echo "  - dest is the address the Pi must hold to receive (or 255.255.255.255 broadcast)."
echo
echo "If you saw NOTHING: either link/cabling is bad, or the lidar streams to a"
echo "specific dest IP the switch won't forward — try the broadcast check:"
echo "  sudo timeout 6 tcpdump -ni $IFACE -c 4 udp"
echo
echo ">>> Decision: to avoid colliding with wifi (wlan0 = 192.168.1.0/24), we move"
echo "    the Pandar to an ISOLATED subnet 192.168.3.x via its web UI, then apply"
echo "    the netplan in 03_eth0_static_netplan.yaml. See that file's header."
