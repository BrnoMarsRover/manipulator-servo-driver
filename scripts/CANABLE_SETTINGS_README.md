To properly setup CAN over canable v2

1) Move the 80-canable.rules file to /etc/udev/rules.d/80-canable.rules
2) udevadm control --reload-rules && udevadm trigger
3) sudo slcand -o -c -s6 /dev/canable_v2 can0
4) sudo ifconfig can0 up
5) You should be set up correctly - print the CAN communication with "candump can0"