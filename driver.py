import os
import sys
import serial
import socket
import netifaces
import json
import multiprocessing

localhost = '127.0.0.1'
kernel_proc_state = False
kernel_agent_work = True

firedet_en = '0' # activate autogun
robot_forw = '1' # moving forward
robot_back = '2' # moving backward
robot_right = '3' # rotate robot right
robot_left = '4' # rotate robot left
gun_up = '5' # gun up
gun_down = '6' # gun down
gun_left = '7' # gun left
gun_right = '8' # gun right
pump_on = 'p' # pump on
stop = 's'
power_off = 'e'


def connect_to_cvkernel_state_sock(settings):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(settings['state_unix_dst'])
    return sock

def create_metadata_udp_server(settings):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(localhost, settings['meta_udp_port'])
    return sock

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

def cvkernel_agent(uart, proc_state, agent_work, image_resolution):
    enable_flag_sent = False
    pump_on = False
    state_socket, metadata_socket = connect_to_cvkernel(settings)
    while agent_work:
        if proc_state:
            if not enable_flag_sent:
                state_socket.send('e')
                enable_flag_sent = True
            input_rects = metadata_socket.recv(2048)
            rect_size = input_rects[0] << 8 + input_rects[1]
            rects = []
            for i in range(0, rect_size):
                x = input_rects[i * 24 + 1] << 8 + input_rects[i * 24 + 2]
                y = input_rects[i * 24 + 3] << 8 + input_rects[i * 24 + 4]
                w = input_rects[i * 24 + 5] << 8 + input_rects[i * 24 + 6]
                h = input_rects[i * 24 + 7] << 8 + input_rects[i * 24 + 8]
                rects.append((x, y, w, h))
            sorted_rects = sorted(rects, key=lambda r: (r[2] - r[0]) * (r[3] - r[1]), reverse=True)
            biggest_rect = sorted_rects[0]
            x, y, w, h = biggest_rect
            cx = image_resolution[0] / 2
            cy = image_resolution[1] / 2
            if x <= cx <= x + w and y <= cy <= y + h:
                uart.write('p')
                pump_on = True
            elif pump_on:
                uart.write('s')
                pump_on = False

            command = ''
            if cy <= y:
                command += gun_up
            elif cy >= y + h:
                command += gun_down
            if cx <= x:
                command += gun_right
            else:
                command += gun_left
            uart.write(command)
        else:
            if enable_flag_sent:
                state_socket.send('s')
                enable_flag_sent = False
    state_socket.send('c')
    state_socket.close()
    metadata_socket.close()
    return

if __name__ == '__main__':
    json_file_path, cvkernel_comm, cvkernel_args = parse_args()
    cvkernel_pid = os.fork()
    if cvkernel_pid > 0:
        uart = serial.Serial("/dev/ttyACM0", 9600)
        settings = json.load(open(json_file_path, 'r'))
        gamepad_conn = connect_to_gamepad(settings)
        image_resolution = (settings['im_width'], settings['im_height'])
        proc = multiprocessing.Process(target=cvkernel_agent, args=(uart, kernel_proc_state, kernel_agent_work, image_resolution))
        while 1:
            com = gamepad_conn.recv(1)
            if com is firedet_en:
                kernel_proc_state = True
            else:
                kernel_proc_state = False
            uart.write(com)
            if com is power_off:
                kernel_agent_work = False
                break

        disconnect()
        os.kill(cvkernel_pid, 9)
    elif cvkernel_pid == 0:
        os.execv(cvkernel_comm, cvkernel_args)  #child
    elif cvkernel_pid == -1:
        print 'cannot create new process'
