import serial



if __name__ == '__main__':
    num = 1
    it = 1
    uart = serial.Serial("/dev/ttyACM0", 9600)
    while True:
        print 'incoming ' + uart.readline() + 'it = {}'.format(it)
        if it is 0:
            uart.write(str(num))
            num += 1
        it = (it + 1) % num