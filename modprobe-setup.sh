sudo modprobe peak_usb
<<<<<<< HEAD
sudo modprobe peak_pci
=======
if [ $(hostname) != "rover" ]
then
    sudo modprobe peak_pci
fi
>>>>>>> canbus_local
# sudo modprobe pcan
sudo modprobe esd_usb2

sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 1000
