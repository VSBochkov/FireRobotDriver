import pexpect
import bluetooth
import os
import sys
import serial
import time


def main():
    exp = pexpect.spawn('bluetoothctl')
    time.sleep(1)
    exp.expect('#')
    exp.sendline('power on')
    time.sleep(2)
    exp.expect('#')
    exp.sendline('agent on')
    time.sleep(3)
    exp.expect('#')
    exp.send('quit')
    time.sleep(1)
    exp.eof()

    return

if __name__ == '__main__':
    main()
