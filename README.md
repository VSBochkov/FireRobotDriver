# FireRobotDriver

Python project

Key concepts:
programm will start from autoboot
programm will receive commands from bluetooth and executes CVKernel and push commands to arduino via Serial to USB

Key libraries:

Pexpect - enabling bluetooth in start of application

PyBluez - receiving commands from Android device via bluetooth (wrong, Android doesn't support bluez stack)
instead this just use sockets upon wifi connection to nexus hotspot

os      - call fork and exec with CVKernel

sys     - command line args

serial  - send commands to arduino via Serial to USB
