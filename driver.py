import os
import sys
import serial
import time
import socket
import netifaces



def connect_to_server():
    sock = socket.socket()
    self_addr = netifaces.ifaddresses('wlan0')[netifaces.AF_INET][0]['addr']
    serv_addr = self_addr[:self_addr.rfind('.')] + '1'
    sock.connect(serv_addr, 8989)
    return sock

def disconnect():
    return

def main():
    sock = connect_to_server()
    while 1:
        com = sock.recv(1)
        if com is '0':
            # activate autogun
        elif com is '1':
            # moving forward
        elif com is '2':
            # moving backward
        elif com is '3':
            # rotate robot right
        elif com is '4':
            # rotate robot left
        elif com is '5':
            # gun up
        elif com is '6':
            # gun down
        elif com is '7':
            # gun left
        elif com is '8':
            # gun right
        elif com is 'p':
            # pump on
        elif com is 's':
            # stop

        if com is not '0':
            # stop autogun

        if com is 'e':
            break

    disconnect()
    return

if __name__ == '__main__':
    main()
