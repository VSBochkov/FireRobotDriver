import serial               # Подключение модуля для обмена данными с Arduino через UART
import json                 # Подключение модуля для сериализации и десериализации данных в формате JSON
import multiprocessing      # Подключение модуля для организации многопоточности
import Queue                # Подключение модуля для перехвата исключений при операциях с очередями
import os                   # Подключение модуля для использования переменных окружения
import math                 # Подключение модуля для использования методов округления
import time                 # Подключение модуля для использования функции паузы потока

from cv_kernel import cv_client                 # Подключение модуля клиента ядра из библиотеки платформы видеоаналитики
from cv_kernel import cv_network_controller     # Подключения модуля сетевого менеджера из библиотеки платформы видеоаналитики


class FireRobotDriver:  # Определение класса драйвера
    autogun_on = 1      # Состояние активации автоприцеливания
    autogun_off = 2     # Состояние деактивации автоприцеливания
    client_closed = 3   # Состояние деактивации клиента ядра
    meta = 4            # Состояние приема метаданных
    gamepad_found = 5   # Состояние нахождения пульта управления роботом
    robot_forw = '1'    # Команда движения шасси робота вперед
    robot_back = '2'    # Команда движения шасси робота назад
    robot_right = '3'   # Команда поворота шасси робота направо
    robot_left = '4'    # Команда поворота шасси робота налево
    gun_up = '5'        # Команда вращения пушки робота вверх
    gun_down = '6'      # Команда вращения пушки робота вниз
    gun_left = '7'      # Команда вращения пушки робота влево
    gun_right = '8'     # Команда вращения пушки робота вправо
    pump_on = 'p'       # Команда включения насоса
    pump_off = 'e'      # Команда выключения насоса
    stop = 's'          # Команда остановки предыдущей команды
    autogun = 'a'       # Команда активации автоприцеливания

    def __init__(self):     # Конструктор класса
        self.driver_queue = multiprocessing.Queue()     # Создание очереди команд управления роботом
        self.driver = multiprocessing.Process(target=self.__driver)     # Создание процесса управления роботом
        fire_robot_driver_path = os.environ.get('FIRE_ROBOT_DRIVER_PATH')   # Инициализация пути до дирректории с исходным кодом
        self.gamepad_settings = json.load(open(fire_robot_driver_path + '/gamepad.json', 'r')) # Загрузка настроек пульта управления роботом
        self.network_controller = cv_network_controller(self.__gamepad_found)   # Создание менеджера сети
        self.network_controller.add_mac_handler(self.gamepad_settings['mac_address'])   # Добавление MAC адреса пульта управления, для поиска и подключения к нему
        self.cv_client = cv_client(     # Создание клиента ядра
            network_controller=self.network_controller,     # Передача сетевого менеджера
            cvkernel_json_path=fire_robot_driver_path + '/cv_kernel_settings.json', # Передача пути до настроек ядра
            run_state_handler=self.__vproc_run,         # Инициализация обработчиков
            ready_state_handler=self.__vproc_ready,     # Состояний
            closed_state_handler=self.__vproc_closed,   # Клиента ядра
            meta_handler=self.__metadata_received,      # Инициализация обработчика метаданных
            cvproc_json_path=fire_robot_driver_path + '/fire_overlay_on_web.json'   # Передача пути до спецификации видеоаналитики
        )

        resolution = tuple(self.cv_client.cv_process_description['proc_resolution'].split('x')) # Инициализация
        self.image_resolution = int(resolution[0]), int(resolution[1])                          # разрешения изображения
        self.fps = float(self.cv_client.cv_process_description['fps'])  # Инициализация количества кадров в секунду
        self.uart = serial.Serial("/dev/ttyACM0", 9600)     # Инициализация UART соединения с платой Arduino
        self.gamepad_tcp = None     # TCP сокет соединения с пультом управления
        self.biggest_rect = None    # Наиболее подходящий прямоугольник для прицеливания
        self.ema_aimed = 0.         # Значение скользящего среднего прицеливания
        self.aver_left = 0.         # Значение среднего смещения дула влево
        self.aver_right = 0.        # Значение среднего смещения дула вправо
        self.aver_up = 0.           # Значение среднего поднятия дула вверх
        self.aver_down = 0.         # Значение среднего опускания дула вниз
        self.aimed_thresh = 0.2     # Значение порога прицеливания для активации насоса
        self.move_thresh = 0.4      # Значение порога смещения дула в сторону целевой прямоугольной области очага пожара
        self.pump_active = False    # Флаг активности насоса
        self.meta_iteration = 0     # Итерация полученных метаданных
        self.second = math.ceil(self.fps)   # Количество итераций в секунде
        self.driver.run()           # Вызов метода активации процесса драйвера

    def __gamepad_found(self, mac, ip_address):     # Метод-обработчик нахождения пульта управления в сети
        self.driver_queue.put({'type': FireRobotDriver.gamepad_found, 'ip_address': ip_address})    # Помещаем в очередь сообщение о нахождении пульта управления

    def __vproc_run(self):      # Метод-обработчик изменения состояния клиента на значение "На исполнении"
        self.driver_queue.put({'type': FireRobotDriver.autogun_on})     # Помещаем в очередь сообщение о активации режима автоприцеливания

    def __vproc_ready(self):    # Метод-обработчик изменения состояния клиента на значение "Остановлен"
        self.driver_queue.put({'type': FireRobotDriver.autogun_off})    # Помещаем в очередь сообщение о деактивации режима автоприцеливания

    def __vproc_closed(self):   # Метод-обработчик изменения состояния клиента на значение "Деактивирован"
        self.driver_queue.put({'type': FireRobotDriver.client_closed})  # Помещаем в очередь сообщение о закрытии клиента ядра

    @staticmethod       # Статический метод класса
    def __rect_area(rect):  # вычисления площади прямоугольника
        return rect['w'] * rect['h']    # Возврат произведения ширины на высоту прямоугольника

    @staticmethod       # Статический метод класса
    def __rect_int(r1, r2):     # вычисления пересечения прямоугольников
        x1 = max(r1['x'], r2['x'])  # координата x стартовой точки прямоугольника равна максимуму между стартовыми координатами x двух прямоугольников
        y1 = max(r1['y'], r2['y'])  # координата y стартовой точки прямоугольника равна максимуму между стартовыми координатами y двух прямоугольников
        x2 = min(r1['x'] + r1['w'], r2['x'] + r2['w'])  # координата x конечной точки прямоугольника равна минимуму между конечными координатами x двух прямоугольников
        y2 = min(r1['y'] + r1['h'], r2['y'] + r2['h'])  # координата y конечной точки прямоугольника равна минимуму между конечными координатами y двух прямоугольников
        if x1 >= x2 or y1 >= y2:    # если стартовые координаты пересечения больше конечных
            return {'x': 0, 'y': 0, 'w': 0, 'h': 0}     # то пересечения нет - возвращаем нулевой прямоугольник
        else:                       # иначе
            return {'x': x1, 'y': y1, 'w': x2 - x1, 'h': y2 - y1}   # возвращаем вычисленный прямоугольник

    def __near_rect_metric(self, rect):     # Метод вычисления значения метрики ближайшего прямоугольника к целевому
        int_rect = FireRobotDriver.__rect_int(rect, self.biggest_rect)  # Находим пересечение текущего прямоугольника с наиближайшим с предыдущего кадра
        int_area = FireRobotDriver.__rect_area(int_rect)            # Находим значение площади пересечения
        big_area = FireRobotDriver.__rect_area(self.biggest_rect)   # Находим значение площади наиближайшего прямоугольника
        if int_area > 0:            # Если пересечение существует
            return 1. + int_area / big_area     # То возвращаем большую метрику для прямоугольника
        else:   # Иначе
            return FireRobotDriver.__rect_area(rect) / big_area     # Возвращаем меньшую метрику для прямоугольника

    def __get_biggest_rect(self, rects):    # Метод нахождения наиближайшего прямоугольника
        biggest_rect = None     # Инициализируем результат нулем
        max_metric = 0.         # Инициализируем значение максимальной метрики нулем
        if self.biggest_rect is None:   # Если наиближайший прямоугольник не был инициализирован ранее
            for rect in rects:          # То для всех текущих прямугольников
                metric = FireRobotDriver.__rect_area(rect)  # вычисляем его площадь
                if max_metric < metric:    # И если она больше максимальной метрики
                    max_metric = metric    # Обновляем максимальную метрику и
                    biggest_rect = rect    # Сохраняем значение текущего прямоугольника как наиближайшего
        else:       # Если наиближайший прямоугольник не был инициализирован ранее
            for rect in rects:  # То для всех прямоугольников
                metric = self.__near_rect_metric(rect)  # находим значение метрики сопоставления прямоугольников на кадре
                if max_metric < metric:    # И если она больше максимальной метрики
                    max_metric = metric    # Обновляем максимальную метрику и
                    biggest_rect = rect    # Сохраняем значение текущего прямоугольника как наиближайшего

        return biggest_rect     # Возвращаем наиближайший прямоугольник

    def __metadata_received(self, metadata):    # Обработчик входящих метаданных
        self.driver_queue.put({'type': FireRobotDriver.meta, 'meta': metadata})     # Вставляем в очередь сообщение о принятии метаданных

    def __drive_gun(self, meta):    # Метод реализации автоприцеливания
        if 'FlameSrcBBox' not in meta.keys():   # Если в метаданных нет списка прямоугольных областей очагов пожара
            self.ema_aimed = float(self.ema_aimed * self.fps * 3) / float(self.fps * 3 + 1.)    # Уменьшаем среднее активации помпы
            if self.ema_aimed < self.aimed_thresh and self.pump_active: # Если среднее прицеливания меньше порога и помпа была активирована
                self.uart.write(FireRobotDriver.pump_off)   # Посылаем команду остановки насоса
                self.uart.read(1)           # Прочитываем ответ от Arduino
                self.pump_active = False    # Флаг работы помпы инициализируем значением "Остановлен"
            return  # Возврат из функции

        self.meta_iteration += 1        # увеличиваем номер итерации
        rects = meta['FlameSrcBBox']['bboxes']  # Извлекаем массив прямоугольных областей очагов пожара из метаданных
        self.biggest_rect = self.__get_biggest_rect(rects)  # Находим наиближайший прямоугольник
        if self.biggest_rect is None:   # Если он не инициализирован
            return  # Возврат из функции

        x, y, w, h = self.biggest_rect['x'], self.biggest_rect['y'], self.biggest_rect['w'], self.biggest_rect['h'] # Считываем компоненты наиближайшего прямоугольника
        cx = int(self.image_resolution[0] / 2)      # Инициализируем координаты
        cy = int(self.image_resolution[1] / 2)      # центра кадра
        r_cx = int(x + w / 2)                       # Инициализируем координаты
        r_cy = int(y + h / 2)                       # центра наиближайшей прямоугольной области
        d_rx = int(self.image_resolution[0] / 20)   # Инициализируем
        d_ry = int(self.image_resolution[1] / 20)   # интервалы области смещения прицела
        aimed = int((x <= cx <= x + w) and (y <= cy <= y + h))  # Устанавливаем флаг прицеливания как нахождения центра кадра внутри прямоугольника
        self.ema_aimed = float(self.ema_aimed * self.fps + aimed) / float(self.fps + 1.)    # Пересчет среднего компонента наведения прицела
        command = ''    # Определяем строку команд на мехатронную часть робота
        if self.ema_aimed >= self.aimed_thresh + 0.2 and not self.pump_active:  # Если среднее прицеливания больше порога и помпа деактивирована
            command += FireRobotDriver.pump_on  # Активируем помпу
            self.pump_active = True             # Устанавливаем флаг активности насоса
        elif self.ema_aimed < self.aimed_thresh and self.pump_active:   # Если среднее прицеливания меньше порога и помпа активирована
            command += FireRobotDriver.pump_off # Останавливаем насос
            self.pump_active = False            # Устанавливаем флаг неактивности насоса

        self.aver_down += float(cy < r_cy - d_ry)   # Аккумулируем
        self.aver_up += float(cy > r_cy + d_ry)     # средние
        self.aver_left += float(cx > r_cx + d_rx)   # компоненты
        self.aver_right += float(cx < r_cx - d_rx)  # смещения прицела в сторону центра очага пожара

        print '__drive_gun: iteration = {}'.format(self.meta_iteration) # Выводим значения текущей итерации
        print '__drive_gun: target bbox: {}'.format(self.biggest_rect)  # обработки метаданных в интерфейс командной строки

        if self.meta_iteration == self.second:      # Если количество итераций равно количеству кадров в одной секунде
            self.aver_down = float(self.aver_down / float(self.second))     # Усредняем
            self.aver_up = float(self.aver_up / float(self.second))         # аккумулируемые
            self.aver_left = float(self.aver_left / float(self.second))     # компоненты
            self.aver_right = float(self.aver_right / float(self.second))   # смещения прицела
            print '__drive_gun: aver_down = {}, aver_up = {}, aver_left = {}, aver_right = {}'.format(  # Выводим значения
                self.aver_down, self.aver_up, self.aver_left, self.aver_right   # компонент смещения
            )   # прицела в интерфейс командной строки
            move_y = ((float(self.aver_down) > self.move_thresh) or (float(self.aver_up) > self.move_thresh))       # Генерируем команды
            move_x = ((float(self.aver_right) > self.move_thresh) or (float(self.aver_left) > self.move_thresh))    # поворота дула в очаг пожара
            if move_x:  # Если принято решение поворота дула по азимуту
                if self.aver_down > self.aver_up:           # То определяем
                    command += FireRobotDriver.gun_down     # направление
                else:                                       # смещения
                    command += FireRobotDriver.gun_up       # прицела
            if move_y:  # Если принято решение поворота дула по высоте
                if self.aver_left > self.aver_right:        # То определяем
                    command += FireRobotDriver.gun_left     # направление
                else:                                       # смещения
                    command += FireRobotDriver.gun_right    # прицела
            self.aver_down = 0.     # Сбрасываем средние
            self.aver_up = 0.       # характеристики
            self.aver_right = 0.    # поворота
            self.aver_left = 0.     # дула
            self.meta_iteration = 0 # Сбрасываем счетчик итераций

        print '__drive_gun: ema: aimed = {}'.format(self.ema_aimed)     # Вывод характеристики наведения в интерфейс командной строки

        if len(command) > 0:    # Если список команд для управления мехатронной частью не пуст
            print '__drive_gun: send {} to arduino'.format(command) # То вывод команд посылаемых на Arduino
            for char in command:        # Пересчет в цикле всех команд списка
                self.uart.write(char)   # Посылаем команду на Arduino
                self.uart.read(1)       # принимаем результат обработки команды
                time.sleep(0.001)       # останавливаем поток на 1 мс

    def __driver(self):         # Метод потока работы драйвера
        client_was_run = False  # Инициализируем
        autogun_run = False     # начальное состояние
        driver_running = True   # управляющих флагов

        while driver_running:   # Организуем цикл пока значение работы потока истинно
            try:        # Секция перехвата исключений
                packet = self.driver_queue.get_nowait()     # Получаем текущий пакет из очереди управления драйвером без блокировки потока
            except Queue.Empty: # Если очередь пуста
                pass    # то ничего не делаем
            else:   # Если пакет получен
                if packet['type'] == FireRobotDriver.gamepad_found:     # Если тип пакета - найден пульт управления
                    print '__driver: gamepad found'     # То выводим эту информацию в интерфейс командной строки
                    self.gamepad_tcp = cv_network_controller.connect_to_tcp_host(   # Вызываем метод
                        packet['ip_address'], self.gamepad_settings['port']         # присоединения к пульту
                    )                                                               # по TCP протоколу
                    print '__driver: gamepad connected at: {}'.format(self.gamepad_tcp.getpeername())   # Выводим сообщение о присоединении геймпада
                elif packet['type'] == FireRobotDriver.autogun_on:  # Если тип пакета - активация режима автоприцеливания
                    print '__driver: Autogun active'        # То выводим эту информацию в интерфейс командной строки
                    autogun_run = True                      # Активируем статус работы алгоритма автоприцеливания
                    client_was_run = True                   # Активируем статус работы клиента ядра видеоаналитики
                elif packet['type'] == FireRobotDriver.autogun_off: # Если тип пакета - деактивация режима автоприцеливания
                    print '__driver: Autogun inactive'      # То выводим эту информацию в интерфейс командной строки
                    autogun_run = False                     # Сбрасываем статус работы алгоритма автоприцеливания
                    self.biggest_rect = None                # Сбрасываем наиближайший прямоугольник для наведения прицела
                elif packet['type'] == FireRobotDriver.meta and autogun_run:    # Если тип пакета - метаданные и режим автоприцеливания активирован
                    self.__drive_gun(packet['meta'])        # Вызываем метод выполнения итерации автоприцеливания
                elif packet['type'] == FireRobotDriver.client_closed:   # Если тип пакета - закрытие клиента ядра
                    print '__driver: cv_client closed'      # То выводим эту информацию в интерфейс командной строки
                    if client_was_run:                      # Если клиент был активирован
                        driver_running = False              # Сбрасываем флаг работы драйвера
                        client_was_run = False              # Сбрасываем флаг активации клиента
            finally:    # Выполняем независимо от исключений
                if self.gamepad_tcp is not None:    # Если TCP соединение до пульта управления установлено
                    command = cv_network_controller.async_receive_byte(self.gamepad_tcp)    # Пытаемся принять байт команды от пульта управления
                    if command == cv_network_controller.connection_is_broken:   # Если соединение было прервано
                        print '__driver: gamepad disconnected, wait for connection is established again'    # То выводим эту информацию в интерфейс командной строки
                        gamepad_host = self.gamepad_tcp.getpeername()   # Получаем IP-адресс и порт пульта управления
                        self.gamepad_tcp.close()                        # Закрываем текущее соединение
                        self.gamepad_tcp = cv_network_controller.connect_to_tcp_host(gamepad_host[0], gamepad_host[1])  # Переустанавливаем соедиенние до пульта управления
                        print '__driver: gamepad has been connected'    # Выводим информацио о подключении пульта управления
                    elif command == cv_network_controller.no_data:      # Если нет команд от пульта
                        pass        # То ничего не делаем
                    else:       # Если приняли команду от пульта управления
                        print '__driver: command is: {}'.format(command)    # То выводим эту информацию в интерфейс командной строки
                        if command == FireRobotDriver.autogun:      # Если значение команды "автоприцеливание"
                            if autogun_run:             # Если автоприцеливание было активировано
                                self.cv_client.stop()   # Останавливаем его
                                print '__driver: stop autogun'  # и выводим в интерфейс командной строки
                            else:                       # Если оно не было активировано
                                self.cv_client.run()    # То активируем его
                                print '__driver: run autogun'   # и выводим в интерфейс командной строки
                        self.uart.write(command)    # Передаем команду от пульта управления на плату Arduino
                        self.uart.read(1)           # Прочитываем байт ответа
        self.network_controller.stop()      # Останавливаем сетевой менеджер
        print 'exit from __driver'          # Выводим сообщение об остановке драйвера


def run():      # Метод активации драйвера
    FireRobotDriver()   # Создаем объект драйвера

if __name__ == '__main__':  # Если файл запущен из интерпретатора
    run()       # Вызываем метод активации драйвера