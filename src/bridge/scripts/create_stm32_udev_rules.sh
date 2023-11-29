#!/bin/bash

echo "remap the device serial port(ttyACMX) to  stm32"
echo "stm32 usb connection as /dev/stm32 , check it using the command : ls -l /dev|grep ttyACM"
echo "start copy stm32.rules to  /etc/udev/rules.d/"
sudo cp scripts/stm32.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
