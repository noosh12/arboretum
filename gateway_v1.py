import serial 
import pprint
import xbee

PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

print('Running..')
print('PLease WOr...')

usb_port = serial.Serial(PORT,BAUD_RATE)

xbee1 = xbee(usb_port)

