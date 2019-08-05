#!/usr/bin/python
# coding=utf-8
from __future__ import print_function
import serial, struct, sys, time
import Adafruit_DHT
import smbus
import mysql.connector as mysql

DHT22Sensor = Adafruit_DHT.DHT22
DHTpin = 16
 
DEBUG = 0
CMD_MODE = 2
CMD_QUERY_DATA = 4
CMD_DEVICE_ID = 5
CMD_SLEEP = 6
CMD_FIRMWARE = 7
CMD_WORKING_PERIOD = 8
MODE_ACTIVE = 0
MODE_QUERY = 1
I2C_ADDR  = 0x3F # I2C device address
LCD_WIDTH = 16   # Maximum characters per line
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line
LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off
ENABLE = 0b00000100 # Enable bit
E_PULSE = 0.0005
E_DELAY = 0.0005

bus = smbus.SMBus(1) # Rev 2 Pi uses 1
 
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 9600
 
ser.open()
ser.flushInput()
 
byte, data = 0, ""

db = mysql.connect(host="localhost", user="python", passwd="pythonpassword", database="dust")
cursor=db.cursor()

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
 
def dump(d, prefix=''):
    print(prefix + ' '.join(x.encode('hex') for x in d))
 
def construct_command(cmd, data=[]):
    assert len(data) <= 12
    data += [0,]*(12-len(data))
    checksum = (sum(data)+cmd-2)%256
    ret = "\xaa\xb4" + chr(cmd)
    ret += ''.join(chr(x) for x in data)
    ret += "\xff\xff" + chr(checksum) + "\xab"
 
    if DEBUG:
        dump(ret, '> ')
    return ret
 
def process_data(d):
    r = struct.unpack('<HHxxBB', d[2:])
    pm25 = r[0]/10.0
    pm10 = r[1]/10.0
    checksum = sum(ord(v) for v in d[2:8])%256
    return [pm25, pm10]
    #print("PM 2.5: {} μg/m^3  PM 10: {} μg/m^3 CRC={}".format(pm25, pm10, "OK" if (checksum==r[2] and r[3]==0xab) else "NOK"))
 
def process_version(d):
    r = struct.unpack('<BBBHBB', d[3:])
    checksum = sum(ord(v) for v in d[2:8])%256
    print("Y: {}, M: {}, D: {}, ID: {}, CRC={}".format(r[0], r[1], r[2], hex(r[3]), "OK" if (checksum==r[4] and r[5]==0xab) else "NOK"))
 
def read_response():
    byte = 0
    while byte != "\xaa":
        byte = ser.read(size=1)
 
    d = ser.read(size=9)
 
    if DEBUG:
        dump(d, '< ')
    return byte + d
 
def cmd_set_mode(mode=MODE_QUERY):
    ser.write(construct_command(CMD_MODE, [0x1, mode]))
    read_response()
 
def cmd_query_data():
    ser.write(construct_command(CMD_QUERY_DATA))
    d = read_response()
    values = []
    if d[1] == "\xc0":
        values = process_data(d)
    return values
 
def cmd_set_sleep(sleep=1):
    mode = 0 if sleep else 1
    ser.write(construct_command(CMD_SLEEP, [0x1, mode]))
    read_response()
 
def cmd_set_working_period(period):
    ser.write(construct_command(CMD_WORKING_PERIOD, [0x1, period]))
    read_response()
 
def cmd_firmware_ver():
    ser.write(construct_command(CMD_FIRMWARE))
    d = read_response()
    process_version(d)
 
def cmd_set_id(id):
    id_h = (id>>8) % 256
    id_l = id % 256
    ser.write(construct_command(CMD_DEVICE_ID, [0]*10+[id_l, id_h]))
    read_response()
 
if __name__ == "__main__":
    lcd_init()
    lcd_string("Air quality     ",LCD_LINE_1)
    lcd_string("monitor         ",LCD_LINE_2)

    try:
        print("Turning fan on")
        cmd_set_sleep(0)
        cmd_set_mode(1)
        time.sleep(2)
        pmvalues = cmd_query_data();
        time.sleep(2)
        pm25 = 0.0
        pm10 = 0.0
        temp = 0.0
        hum = 0.0
        count = 0
        lcd_string("Measuring       ",LCD_LINE_1)
        lcd_string("                ",LCD_LINE_2)
        for t in range(5):
            pmvalues = cmd_query_data();
            humidity, temperature = Adafruit_DHT.read_retry(DHT22Sensor, DHTpin)
            if humidity < 0 or humidity > 100:
                humidity = None
                print("Erronous humidity reading")
            lcdline = "Sample "+str(count+1)+"/5      "
            lcd_string(lcdline,LCD_LINE_2)
            if pmvalues is not None and humidity is not None and temperature is not None:
                pm25 += pmvalues[0]
                pm10 += pmvalues[1]
                temp += temperature
                hum += humidity
                count += 1
                print("PM2.5: {:.1f}, PM10: {:.1f}, Temp: {:.1f}*C,  Humidity: {:.1f}%".format(pmvalues[0], pmvalues[1], temperature, humidity))
            else:
                print("Error getting air quality data")
            time.sleep(3)

        if count > 0:
            pm25 /= count
            pm10 /= count
            temp /= count
            hum /= count
            print("AVERAGE: PM2.5={:.1f}, PM10={:.1f}, T={:.1f}C, H={:.1f}%".format(pm25, pm10, temp, hum))
 
            lcdline = "T{:2.1f}C 2.5u:{:3.1f}    ".format(temp, pm25) 
            lcd_string(lcdline[:16], LCD_LINE_1)
            lcdline = "H{:2.1f}% 10u:{:3.1f}    ".format(hum,pm10) 
            lcd_string(lcdline[:16], LCD_LINE_2)
            dbquery = "INSERT INTO airquality (temperature, humidity, u10, u25) values (%s, %s, %s, %s)"
            dbvalues = ("{:.1f}".format(temp), "{:.1f}".format(hum), "{:.1f}".format(pm10), "{:.1f}".format(pm25))
#            print(dbquery)
#            print(dbvalues)
            cursor.execute(dbquery, dbvalues)
            db.commit()
#        time.sleep(30)

    except KeyboardInterrupt:
        pass
    finally:
        print("Going to sleep...")
        cmd_set_mode(0)
        cmd_set_sleep()
#        lcd_byte(0x01, LCD_CMD)
#        LCD_BACKLIGHT = 0x00  # Off
#        lcd_init()

