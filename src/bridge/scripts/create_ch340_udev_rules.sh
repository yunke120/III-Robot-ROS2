#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ch340"
echo "ch340 usb connection as /dev/ch340 , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy ch340.rules to  /etc/udev/rules.d/"
sudo cp scripts/ch340.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
