#!/bin/bash
# setup a virtual can session, valid until computer restart
# use "candump canRovus" to intercept messages or the cansend_helper.py to send messages

set -e

sudo modprobe vcan

if ! ip link show canRovus &>/dev/null; then
    sudo ip link add dev canRovus type vcan
fi

sudo ip link set up canRovus