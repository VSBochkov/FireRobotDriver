import serial
import json
import multiprocessing
import Queue
import os
import math

from cv_kernel import cv_client
from cv_kernel import cv_network_controller

'''
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
power_off = 'z'
cvproc_enabled = 'e'
cvproc_disabled = stop


def connect_to_cvkernel_state_sock(settings):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((localhost, settings['state_tcp_port']))
    print 'TCP: connected to cvkernel'
    return sock


def create_metadata_udp_server(settings):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((localhost, settings['meta_udp_port']))
    return sock


def connect_to_cvkernel(settings):
    return connect_to_cvkernel_state_sock(settings), create_metadata_udp_server(settings)


def connect_to_gamepad(settings):
    sock = socket.socket()
    self_addr = netifaces.ifaddresses('wlan0')[netifaces.AF_INET][0]['addr']
    serv_addr = self_addr[:self_addr.rfind('.') + 1] + '1'
    sock.connect((serv_addr, settings['gamepad_tcp_port']))
    print 'TCP: connected to gamepad'
    return sock


def recv_bboxes(metadata_sock):
    numbers_buf = metadata_sock.recv(2)
    numbers = numbers_buf[0] << 8 + numbers_buf[1]
    print 'recv_bboxes: number of rectangles is {}'.format(numbers)
    return metadata_sock.recv(numbers * 24), numbers

def disconnect():
    return


def parse_args():
    cvkernel_command = 'CVKernel'
    cvkernel_args = ['off']
    if len(sys.argv) > 1:
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


def rect_area(rect):
    return rect[2] * rect[3]


def rect_int(rect1, rect2):
    r1_x, r1_y, r1_w, r1_h = rect1
    r2_x, r2_y, r2_w, r2_h = rect2
    x1 = max(r1_x, r2_x)
    y1 = max(r1_y, r2_y)
    x2 = min(r1_x + r1_w, r2_x + r2_w)
    y2 = min(r1_y + r1_h, r2_y + r2_h)
    if x1 >= x2 or y1 >= y2:
        return 0, 0, 0, 0
    else:
        return x1, y1, x2 - x1, y2 - y1


def near_rect_metric(rect, biggest_rect):
    int_rect = rect_int(rect, biggest_rect)
    int_area = rect_area(int_rect)
    big_area = rect_area(biggest_rect)
    if int_area > 0:
        return 1. + int_area / big_area
    else:
        return rect_area(rect) / big_area


def cvkernelagent(uart, proc_state, agent_work, image_resolution, has_comand_cv, kernel_mtx):
    received_messages_cnt = 0
    enable_flag_sent = False
    pump_on = False
    state_socket, metadata_socket = connect_to_cvkernel(settings)
    pump_time = 0
    biggest_rect = None
    while agent_work.value == 1:
        with kernel_mtx:
            if proc_state.value == 1:
                if not enable_flag_sent:
                    state_socket.send(cvproc_enabled)
                    enable_flag_sent = True
                    print 'CVKernel activation flag is sent'
                input_rects, rect_number = recv_bboxes(metadata_socket)
                print '#{} - number is {}'.format(received_messages_cnt, rect_number)
                received_messages_cnt += 1
                rect_size = input_rects[0] << 8 + input_rects[1]
                rects = [(
                    input_rects[i * 24 + 1] << 8 + input_rects[i * 24 + 2],
                    input_rects[i * 24 + 3] << 8 + input_rects[i * 24 + 4],
                    input_rects[i * 24 + 5] << 8 + input_rects[i * 24 + 6],
                    input_rects[i * 24 + 7] << 8 + input_rects[i * 24 + 8]
                ) for i in range(0, rect_size)]
                sorted_rects = sorted(rects, key=lambda r: r[2] * r[3] if biggest_rect is None else near_rect_metric(r, biggest_rect), reverse=True)
                biggest_rect = sorted_rects[0]
                print 'biggest_rect = {}'.format(biggest_rect)
                x, y, w, h = biggest_rect
                cx = image_resolution[0] / 2
                cy = image_resolution[1] / 2
                if x <= cx <= x + w and y <= cy <= y + h:
                    uart.write(pump_on)
                    pump_on = True
                    pump_time = 0
                elif pump_on:
                    if pump_time > 60:
                        uart.write(stop)
                        pump_on = False
                    else:
                        pump_time += 1

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
            elif enable_flag_sent:
                state_socket.send(cvproc_disabled)
                enable_flag_sent = False
            else:
                has_comand_cv.wait()

    state_socket.close()
    metadata_socket.close()
    return


if __name__ == '__main__':
    kernel_state_mtx = multiprocessing.RLock()
    kernel_proc_state = multiprocessing.Value('b', 0)
    kernel_agent_work = multiprocessing.Value('b', 1)
    kernel_has_command_cv = multiprocessing.Condition(kernel_state_mtx)
    json_file_path, cvkernel_comm, cvkernel_args = parse_args()
    uart = serial.Serial("/dev/ttyACM0", 9600)
    settings = json.load(open(json_file_path, 'r'))
    gamepad_conn = connect_to_gamepad(settings)
    image_resolution = (settings['frame_width'], settings['frame_height'])
    proc = multiprocessing.Process(target=cvkernelagent, args=(
        uart, kernel_proc_state, kernel_agent_work,
        image_resolution, kernel_has_command_cv, kernel_state_mtx
    ))
    proc.start()
    while 1:
        com = gamepad_conn.recv(1)
        with kernel_state_mtx:
            kernel_proc_state.value = 1 if com == firedet_en else 0
            if com == firedet_en:
                kernel_has_command_cv.notify()
                print 'Autogun activated'

        print 'command is {}'.format(com)
        uart.write(com)
        if com is power_off:
            with kernel_state_mtx:
                kernel_agent_work.value = 0
            break

    proc.join()
    gamepad_conn.close()
'''


class FireRobotDriver:
    autogun_on = 1
    autogun_off = 2
    client_closed = 3
    meta = 4
    gamepad_found = 5
    robot_forw = '1'  # moving forward
    robot_back = '2'  # moving backward
    robot_right = '3'  # rotate robot right
    robot_left = '4'  # rotate robot left
    gun_up = '5'  # gun up
    gun_down = '6'  # gun down
    gun_left = '7'  # gun left
    gun_right = '8'  # gun right
    pump_on = 'p'  # pump on
    stop = 's'
    autogun = 'a'  # activate autogun
    power_off = 'z'
    cvproc_enabled = 'e'

    def __init__(self):
        self.driver_queue = multiprocessing.Queue()
        self.driver = multiprocessing.Process(target=self.__driver)
        fire_robot_driver_path = os.environ.get('FIRE_ROBOT_DRIVER_PATH')
        self.gamepad_settings = json.load(open(fire_robot_driver_path + '/gamepad.json', 'r'))
        self.network_controller = cv_network_controller(self.__gamepad_found)
        self.network_controller.add_mac_handler(self.gamepad_settings['mac_address'])
        self.cv_client = cv_client(
            network_controller=self.network_controller,
            cvkernel_json_path=fire_robot_driver_path + '/cv_kernel_settings.json',
            run_state_handler=self.__vproc_run,
            ready_state_handler=self.__vproc_ready,
            closed_state_handler=self.__vproc_closed,
            meta_handler=self.__metadata_received,
            cvproc_json_path=fire_robot_driver_path + '/fire_overlay_on_pc.json'
        )

        resolution = tuple(self.cv_client.cv_process_description['proc_resolution'].split('x'))
        self.image_resolution = int(resolution[0]), int(resolution[1])
        self.fps = float(self.cv_client.cv_process_description['fps'])
        self.uart = serial.Serial("/dev/ttyACM0", 9600)
        self.gamepad_tcp = None
        self.biggest_rect = None
        self.ema_aimed = 0.
        self.ema_left = 0.
        self.ema_right = 0.
        self.ema_up = 0.
        self.ema_down = 0.
        self.aimed_thresh = 0.7
        self.move_thresh = 0.7
        self.pump_active = False
        self.qfps = self.fps / 4
        self.meta_iteration = 0
        self.quater_second = math.ceil(self.qfps)
        self.driver.run()

    def __gamepad_found(self, mac, ip_address):
        self.driver_queue.put({'type': FireRobotDriver.gamepad_found, 'ip_address': ip_address})

    def __vproc_run(self):
        self.driver_queue.put({'type': FireRobotDriver.autogun_on})

    def __vproc_ready(self):
        self.driver_queue.put({'type': FireRobotDriver.autogun_off})

    def __vproc_closed(self):
        self.driver_queue.put({'type': FireRobotDriver.client_closed})

    @staticmethod
    def __rect_area(rect):
        return rect['w'] * rect['h']

    @staticmethod
    def __rect_int(r1, r2):
        x1 = max(r1['x'], r2['x'])
        y1 = max(r1['y'], r2['y'])
        x2 = min(r1['x'] + r1['w'], r2['x'] + r2['w'])
        y2 = min(r1['y'] + r1['h'], r2['y'] + r2['h'])
        if x1 >= x2 or y1 >= y2:
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0}
        else:
            return {'x': x1, 'y': y1, 'w': x2 - x1, 'h': y2 - y1}

    def __near_rect_metric(self, rect):
        int_rect = FireRobotDriver.__rect_int(rect, self.biggest_rect)
        int_area = FireRobotDriver.__rect_area(int_rect)
        big_area = FireRobotDriver.__rect_area(self.biggest_rect)
        if int_area > 0:
            return 1. + int_area / big_area
        else:
            return FireRobotDriver.__rect_area(rect) / big_area

    def __get_biggest_rect(self, rects):
        biggest_rect = None
        max_metric = 0.
        if self.biggest_rect is None:
            for rect in rects:
                metric = FireRobotDriver.__rect_area(rect)
                if max_metric < metric:
                    max_metric = metric
                    biggest_rect = rect
        else:
            for rect in rects:
                metric = self.__near_rect_metric(rect)
                if max_metric < metric:
                    max_metric = metric
                    biggest_rect = rect

        return biggest_rect

    def __metadata_received(self, metadata):
        self.driver_queue.put({'type': FireRobotDriver.meta, 'meta': metadata})

    def __drive_gun(self, meta):
        if 'FlameSrcBBox' not in meta.keys():
            self.ema_aimed = (self.ema_aimed * self.fps) / float(self.fps + 1.)
            self.ema_up = (self.ema_up * self.qfps) / float(self.qfps + 1.)
            self.ema_down = (self.ema_down * self.qfps) / float(self.qfps + 1.)
            self.ema_left = (self.ema_left * self.qfps) / float(self.qfps + 1.)
            self.ema_right = (self.ema_right * self.qfps) / float(self.qfps + 1.)
            return
        self.meta_iteration += 1
        rects = meta['FlameSrcBBox']['bboxes']
        self.biggest_rect = self.__get_biggest_rect(rects)
        if self.biggest_rect is None:
            self.ema_aimed = (self.ema_aimed * self.fps) / float(self.fps + 1.)
            self.ema_up = (self.ema_up * self.qfps) / float(self.qfps + 1.)
            self.ema_down = (self.ema_down * self.qfps) / float(self.qfps + 1.)
            self.ema_left = (self.ema_left * self.qfps) / float(self.qfps + 1.)
            self.ema_right = (self.ema_right * self.qfps) / float(self.qfps + 1.)
            return

        x, y, w, h = self.biggest_rect['x'], self.biggest_rect['y'], self.biggest_rect['w'], self.biggest_rect['h']
        cx = self.image_resolution[0] / 2
        cy = self.image_resolution[1] / 2
        aimed = int(x <= cx <= x + w and y - h <= cy <= y)          # y is reverted [0 at top, frame height at bottom]
        self.ema_aimed = float(self.ema_aimed * self.fps + aimed) / float(self.fps + 1.)
        command = ''
        if self.ema_aimed >= self.aimed_thresh and not self.pump_active:
            command += FireRobotDriver.pump_on
            self.pump_active = True
        elif self.ema_aimed < self.aimed_thresh and self.pump_active:
            command += FireRobotDriver.pump_on
            self.pump_active = False

        self.ema_up = float(self.ema_up * self.qfps + int(cy <= y - h)) / (self.qfps + 1)
        self.ema_down = float(self.ema_down * self.qfps + int(cy >= y)) / (self.qfps + 1)
        self.ema_right = float(self.ema_right * self.qfps + int(cx <= x)) / (self.qfps + 1)
        self.ema_left = float(self.ema_left * self.qfps + int(cx >= x + w)) / (self.qfps + 1)

        if self.meta_iteration == self.quater_second:
            if self.ema_up > self.move_thresh or self.ema_down > self.move_thresh:
                command += FireRobotDriver.gun_up if self.ema_up > self.ema_down else FireRobotDriver.gun_down
            elif self.ema_left > self.move_thresh or self.ema_right > self.move_thresh:
                command += FireRobotDriver.gun_left if self.ema_left > self.ema_right else FireRobotDriver.gun_right
            self.meta_iteration = 0

        print 'iteration = {}'.format(self.meta_iteration)
        print 'target bbox: {}'.format(self.biggest_rect)
        print 'ema: aimed = {}, up = {}, down = {}, left = {}, right = {}'.format(
            self.ema_aimed, self.ema_up, self.ema_down, self.ema_left, self.ema_right
        )
        print 'command is {}'.format(command)

        if len(command) > 0:
            self.uart.write(command)


    '''def __drive_gun(self, packet):
        rects = packet['FlameSrcBBox']['bboxes']
        self.biggest_rect = self.__get_biggest_rect(rects)
        x, y, w, h = self.biggest_rect['x'], self.biggest_rect['y'], self.biggest_rect['w'], self.biggest_rect['h']
        cx = self.image_resolution[0] / 2
        cy = self.image_resolution[1] / 2
        if x <= cx <= x + w and y <= cy <= y + h:
            self.uart.write(FireRobotDriver.pump_on)
            self.pump_active = True
            self.pump_time = 0
        elif self.pump_active:
            if self.pump_time > 60:
                self.uart.write(FireRobotDriver.stop)
                self.pump_active = False
            else:
                self.pump_time += 1

        command = ''
        if cy <= y:
            command += FireRobotDriver.gun_up
        elif cy >= y + h:
            command += FireRobotDriver.gun_down
        if cx <= x:
            command += FireRobotDriver.gun_right
        else:
            command += FireRobotDriver.gun_left
        self.uart.write(command)'''

    def __driver(self):
        client_was_run = False
        autogun_run = False
        driver_running = True
        while driver_running:
            try:
                packet = self.driver_queue.get_nowait()
            except Queue.Empty:
                pass
            else:
                if packet['type'] == FireRobotDriver.gamepad_found:
                    print 'gamepad found'
                    self.gamepad_tcp = cv_network_controller.connect_to_tcp_host(
                        packet['ip_address'], self.gamepad_settings['port']
                    )
                    print 'gamepad connected'
                elif packet['type'] == FireRobotDriver.autogun_on:
                    print 'Autogun active'
                    autogun_run = True
                    client_was_run = True
                elif packet['type'] == FireRobotDriver.autogun_off:
                    print 'Autogun inactive'
                    autogun_run = False
                    self.biggest_rect = None
                elif packet['type'] == FireRobotDriver.meta and autogun_run:
                    self.__drive_gun(packet['meta'])
                elif packet['type'] == FireRobotDriver.client_closed:
                    print 'cv_client closed'
                    if client_was_run:
                        driver_running = False
                        client_was_run = False
            finally:
                if self.gamepad_tcp is not None:
                    command = cv_network_controller.async_receive_byte(self.gamepad_tcp)
                    if command is not None:
                        print 'command is: {}'.format(command)
                        if command == FireRobotDriver.autogun:
                            if autogun_run:             # autogun was activated
                                print 'stop autogun'
                                self.cv_client.stop()   # stop autogun
                            else:                       # autogun was not activated
                                print 'run autogun'
                                self.cv_client.run()    # run autogun
                        self.uart.write(command)
        self.network_controller.stop()
        print 'exit from __driver'


def run():
    FireRobotDriver()

if __name__ == '__main__':
    run()