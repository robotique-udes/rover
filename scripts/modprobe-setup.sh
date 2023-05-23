sudo modprobe peak_usb
# if [ $(hostname) != "rover" ]
# then
#     sudo modprobe peak_pci
# fi
sudo modprobe esd_usb2

sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 1000

# Add the following to /etc/network/interfaces to enable CAN hotplug:
# allow-hotplug can0
# iface can0 can static
#         bitrate 1000000
#         up ip link set $IFACE txqueuelen 1000
