#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading

class WCXC_UART(object):
    
    def __init__(self, baud=115200):
        #self.ser= ''                                             # 串口2
        self.baud = baud                                            # 波特率
        self.mode = 0
        self.uart_get_ok = 0
        self.uart_receive_buf = ''
        self.uart_receive_str = ''
        
        self.ser = serial.Serial(
            port='/dev/arm',
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self.uart_thread = threading.Thread(target=self.serialEvent)
        self.ser.flushInput()
        self.uart_thread.start()
        if self.ser.is_open:
            self.ser.write('uart init ok!\r\n'.encode())
            print('wetro-robot初始化成功，感谢您的使用!\r\n')

            
    #发送字符串 只需传入要发送的字符串即可
    def uart_send_str(self, temp):
        self.ser.write(temp.encode())                   # 串口发送数据

    # 串口接收数据，主要处理数据接受格式，主要格式为<...> {...} $...!  #...! 4种格式，...内容长度不限
    def serialEvent(self):
        try:
            while True:
                if self.uart_get_ok == 0:
                    uart_receive_buf_index = self.ser.inWaiting()
                    if uart_receive_buf_index:
                        self.uart_receive_buf = self.uart_receive_buf+self.ser.read(uart_receive_buf_index).decode("utf-8","ignore")
                        #print('$!:',self.uart_receive_buf, " len:", len(self.uart_receive_buf))
                        if self.mode == 0:
                            if self.uart_receive_buf.find('{') >= 0:
                                self.mode = 1
                            elif self.uart_receive_buf.find('$') >= 0:
                                self.mode = 2
                            elif self.uart_receive_buf.find('#') >= 0:
                                self.mode = 3
                        
                        if self.mode == 1:
                            if self.uart_receive_buf.find('}') >= 0:
                                self.uart_get_ok = 1
                                self.mode = 0
                            
                        elif self.mode == 2:
                            if self.uart_receive_buf.find('!') >= 0:
                                self.uart_get_ok = 2
                                self.mode = 0
            
                                #print('$!:',uart_receive_buf, " len:", len(uart_receive_buf))
                                #print('mode2 end')
                        elif self.mode == 3:
                            if self.uart_receive_buf.find('!') >= 0:
                                self.uart_get_ok = 3
                                self.mode = 0
                         
                                #print('#!:', uart_receive_buf, " len:", len(uart_receive_buf))
                                #print('mode3 end')
                        
                        #print('get2:',uart_receive_buf, " len:", len(uart_receive_buf), " mode:", mode, " getok:", uart_get_ok)
        
        except IOError:
            pass
        
 
#程序入口
if __name__ == '__main__':
    uart = WCXC_UART()                                         # 实例化串口
    uart.uart_send_str('#255P1500T1000!')
    try:
        while 1:
            uart.uart_send_str('#000P2000T1000!')
            time.sleep(1)
            uart.uart_send_str('#000P1000T1000!')
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

    finally:
        uart.ser.close()
        pass




