#whenever the socketcan_interface becomes non-responsive you have, to the
#best of my knowledge, two options: unplug the usb-can adaptor and plug it again
#or run this script. Note besides restarting the interface, it increses the txqueuelen
ip link set can0 down
ip link set can0 up type can bitrate 1000000
ifconfig can0 txqueuelen 1000