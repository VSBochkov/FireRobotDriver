import os
import sys
import serial
import time
import socket
import netifaces
import json

localhost = '127.0.0.1'

def connect_to_cvkernel_state_sock(settings):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(settings['state_unix_dst'])
    return sock

def create_metadata_udp_server(settings):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(localhost, settings['meta_udp_port'])


def connect_to_cvkernel(settings):
    return connect_to_cvkernel_state_sock(settings), create_metadata_udp_server(settings)

def connect_to_gamepad(settings):
    sock = socket.socket()
    self_addr = netifaces.ifaddresses('wlan0')[netifaces.AF_INET][0]['addr']
    serv_addr = self_addr[:self_addr.rfind('.')] + '1'
    sock.connect(serv_addr, settings['gamepad_tcp_port'])
    return sock

def disconnect():
    return

def parse_args():
    cvkernel_command = 'CVKernel'
    cvkernel_args = ['off']
    if sys.argc > 1:
        if sys.argv[1] is 'pi':
            json_file_path = 'fire_overlay_off_rpi.json'
            cvkernel_args.append('rpi')
        else:
            json_file_path = 'fire_overlay_off_pc.json'
            cvkernel_args.append('pc')
    else:
        json_file_path = 'fire_overlay_off_pc.json'
        cvkernel_args.append('pc')

    return json_file_path, cvkernel_command, cvkernel_args

def main():

    return

if __name__ == '__main__':
    json_file_path, cvkernel_comm, cvkernel_args = parse_args()
    cvkernel_pid = os.fork()
    if cvkernel_pid > 0:

        settings = json.load(open(json_file_path, 'r'))
        cvkernel_conn = connect_to_cvkernel(settings)
        gamepad_conn = connect_to_gamepad(settings)
        while 1:
            com = gamepad_conn.recv(1)
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
        os.kill(cvkernel_pid, 9)
    elif cvkernel_pid == 0:
        os.execv(cvkernel_comm, cvkernel_args)  #child
    elif cvkernel_pid == -1:
        print 'cannot create new process'
