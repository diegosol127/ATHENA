import smbus
import time

SLAVE_ADDRESS = 0x70

def write_byte(value):
    bus.write_byte(SLAVE_ADDRESS, value)
    print("Sent to Arduino:", value)

def read_byte():
    received_value = bus.read_byte(SLAVE_ADDRESS)
    print("Receieved from Arduino:", received_value)
    return received_value

bus = smbus.SMBus(1)

try:
    while True:
        read_byte()
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting")