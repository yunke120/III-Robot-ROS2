#!/bin/bash

echo "delete remap the device serial port(ttyACMX) to  stm32"
echo "sudo rm   /etc/udev/rules.d/stm32.rules"
sudo rm   /etc/udev/rules.d/stm32.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
