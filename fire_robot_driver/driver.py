import serial
import json
import multiprocessing
import Queue
import os
import math
import time

from cv_kernel import cv_client
from cv_kernel import cv_network_controller


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
    pump_on = 'p'   # pump on
    pump_off = 'e'  # pump off
    stop = 's'
    autogun = 'a'  # activate autogun
    power_off = 'z'

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
            cvproc_json_path=fire_robot_driver_path + '/fire_overlay_on_web.json'
        )

        resolution = tuple(self.cv_client.cv_process_description['proc_resolution'].split('x'))
        self.image_resolution = int(resolution[0]), int(resolution[1])
        self.fps = float(self.cv_client.cv_process_description['fps'])
        self.uart = serial.Serial("/dev/ttyACM0", 9600)
        self.gamepad_tcp = None
        self.biggest_rect = None
        self.ema_aimed = 0.
        self.aver_left = 0.
        self.aver_right = 0.
        self.aver_up = 0.
        self.aver_down = 0.
        self.aimed_thresh = 0.2
        self.move_thresh = 0.4
        self.pump_active = False
        self.meta_iteration = 0
        self.second = math.ceil(self.fps)
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
            self.ema_aimed = float(self.ema_aimed * self.fps * 3) / float(self.fps * 3 + 1.)
            if self.ema_aimed < self.aimed_thresh and self.pump_active:
                self.uart.write(FireRobotDriver.pump_off)
                self.uart.read(1)
                self.pump_active = False

            return

        self.meta_iteration += 1
        rects = meta['FlameSrcBBox']['bboxes']
        self.biggest_rect = self.__get_biggest_rect(rects)
        if self.biggest_rect is None:
            return

        x, y, w, h = self.biggest_rect['x'], self.biggest_rect['y'], self.biggest_rect['w'], self.biggest_rect['h']
        shift_x = 0     # self.image_resolution[0] * 0.15
        shift_y = 0     # self.image_resolution[1] / 4
        cx = int(self.image_resolution[0] / 2 + shift_x)
        cy = int(self.image_resolution[1] / 2 + shift_y)
        r_cx = int(x + w / 2)
        r_cy = int(y + h / 2)
        d_rx = int(self.image_resolution[0] / 20)
        d_ry = int(self.image_resolution[1] / 20)
        aimed = int((x <= cx <= x + w) and (y <= cy <= y + h))
        self.ema_aimed = float(self.ema_aimed * self.fps + aimed) / float(self.fps + 1.)
        command = ''
        if self.ema_aimed >= self.aimed_thresh + 0.2 and not self.pump_active:
            command += FireRobotDriver.pump_on
            self.pump_active = True
        elif self.ema_aimed < self.aimed_thresh and self.pump_active:
            command += FireRobotDriver.pump_off
            self.pump_active = False

        self.aver_down += float(cy < r_cy - d_ry)
        self.aver_up += float(cy > r_cy + d_ry)
        self.aver_left += float(cx > r_cx + d_rx)
        self.aver_right += float(cx < r_cx - d_rx)

        print '__drive_gun: iteration = {}'.format(self.meta_iteration)
        print '__drive_gun: target bbox: {}'.format(self.biggest_rect)

        if self.meta_iteration == self.second:
            self.aver_down = float(self.aver_down / float(self.second))
            self.aver_up = float(self.aver_up / float(self.second))
            self.aver_left = float(self.aver_left / float(self.second))
            self.aver_right = float(self.aver_right / float(self.second))
            print '__drive_gun: aver_down = {}, aver_up = {}, aver_left = {}, aver_right = {}'.format(
                self.aver_down, self.aver_up, self.aver_left, self.aver_right
            )
            move_y = ((float(self.aver_down) > self.move_thresh) or (float(self.aver_up) > self.move_thresh))
            move_x = ((float(self.aver_right) > self.move_thresh) or (float(self.aver_left) > self.move_thresh))
            if move_x:
                if self.aver_down > self.aver_up:
                    command += FireRobotDriver.gun_down
                else:
                    command += FireRobotDriver.gun_up
            if move_y:
                if self.aver_left > self.aver_right:
                    command += FireRobotDriver.gun_left
                else:
                    command += FireRobotDriver.gun_right
            self.aver_down = 0.
            self.aver_up = 0.
            self.aver_right = 0.
            self.aver_left = 0.
            self.meta_iteration = 0

        print '__drive_gun: ema: aimed = {}'.format(self.ema_aimed)

        if len(command) > 0:
            print '__drive_gun: send {} to arduino'.format(command)
            for char in command:
                self.uart.write(char)
                self.uart.read(1)
                time.sleep(0.001)

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
                    print '__driver: gamepad found'
                    self.gamepad_tcp = cv_network_controller.connect_to_tcp_host(
                        packet['ip_address'], self.gamepad_settings['port']
                    )
                    print '__driver: gamepad connected at: {}'.format(self.gamepad_tcp.getpeername())
                elif packet['type'] == FireRobotDriver.autogun_on:
                    print '__driver: Autogun active'
                    autogun_run = True
                    client_was_run = True
                elif packet['type'] == FireRobotDriver.autogun_off:
                    print '__driver: Autogun inactive'
                    autogun_run = False
                    self.biggest_rect = None
                elif packet['type'] == FireRobotDriver.meta and autogun_run:
                    self.__drive_gun(packet['meta'])
                elif packet['type'] == FireRobotDriver.client_closed:
                    print '__driver: cv_client closed'
                    if client_was_run:
                        driver_running = False
                        client_was_run = False
            finally:
                if self.gamepad_tcp is not None:
                    command = cv_network_controller.async_receive_byte(self.gamepad_tcp)
                    if command == cv_network_controller.connection_is_broken:
                        print '__driver: gamepad disconnected, wait for connection is established again'
                        gamepad_host = self.gamepad_tcp.getpeername()
                        self.gamepad_tcp.close()
                        self.gamepad_tcp = cv_network_controller.connect_to_tcp_host(gamepad_host[0], gamepad_host[1])
                        print '__driver: gamepad has been connected'
                    elif command == cv_network_controller.no_data:
                        pass
                    else:
                        print '__driver: command is: {}'.format(command)
                        if command == FireRobotDriver.autogun:
                            if autogun_run:             # autogun was activated
                                print '__driver: stop autogun'
                                self.cv_client.stop()   # stop autogun
                            else:                       # autogun was not activated
                                print '__driver: run autogun'
                                self.cv_client.run()    # run autogun
                        self.uart.write(command)
                        ans = self.uart.read(1)
        self.network_controller.stop()
        print 'exit from __driver'


def run():
    FireRobotDriver()

if __name__ == '__main__':
    run()