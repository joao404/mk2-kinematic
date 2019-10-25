# mk2-kinematic
MK2 Robotarm Kinematic for Linuxcnc 2.8 or higher

add with:
halcompile --install mk2kins.c

Mathematics are based on https://hackaday.io/project/157951-stm32-robot-arm-controller/log/147424-more-human-friendly-firmware

Added calculations for basemotor and additional offsets.

W0 Offset of the tool ortogonal to the radius of the robotarm.
WR Length-offset of the tool from the last joint
