#!/bin/bash

cp ./scripts/80-canable.rules /etc/udev/rules.d/80-canable.rules
udevadm control --reload-rules && udevadm trigger
sudo slcand -o -c -s4 /dev/canable_v2 can0
sudo ifconfig can0 up