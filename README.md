# FireRobotDriver

Python project

Key concepts:
programm will start from autoboot
programm will receive commands from gamepad (Android) over wifi, executes CVKernel and push commands to arduino via Serial to USB

Key frameworks:

CVKernel
cv_client
python2:
  serial
  os
  sys
  serial
  multiprocessing

For install software to Raspberry PI we have to clone 
CVKernel repository
  build CVKernel (qmake + make from root, sudo python setup.py install from ./python)
This repository:
  sudo python setup.py install
  define enviroment variable FIRE_ROBOT_DRIVER_PATH=path_to_these_source
  redefine mac-address to target (where CVKernel will be run)

For start driver:
  CVKernel;
  fire_robot_driver_run;
