import time

import board
import busio
import math
import neopixel
import random

neo = neopixel.NeoPixel(board.NEOPIXEL, 1)
neo[0] = (10, 0, 0)

ADDR = 0x2A
i2c = busio.I2C(board.SCL, board.SDA)

def try_lock():
    while not i2c.try_lock():
        pass

def led_on():
    i2c.writeto(ADDR, bytes([0x00]), stop = True)

def led_off():
    i2c.writeto(ADDR, bytes([0x01]), stop = True)

def multi_blink(count, duration):
    i2c.writeto(ADDR, bytes([0x02, count, duration]), stop = True)

def pkt_format(lob):
    i2c.writeto(ADDR, bytes([0x10] + lob), stop = True)

def int_to_bytes(n, bits):
    return n.to_bytes((bits + 7) // 8, 'big')

def count_bytes(n):
    cnt = 1
    # print("N : " + str(n))
    while True:
        n = n >> 8
        # print("N : " + str(n))
        if n > 0:
            cnt = cnt + 1
        else:
            break
    return cnt

def data_packet(lon):
    # Length of format
    pkt = b""
    pkt_leng = 0
    crc = 0
    fmt = list()
    for n in lon:
        fmt.append(count_bytes(n))
    # print(fmt)

    # https://gehrcke.de/2014/02/concatenate-byte-strings-in-python-3/
    for ndx in range(len(fmt)):
        v = int_to_bytes(fmt[ndx], 8)
        pkt = pkt + v
        crc = (crc + fmt[ndx]) % 255

    for ndx in range(len(fmt)):
        v = int_to_bytes(lon[ndx], fmt[ndx] * 8)
        # print("n: " + str(lon[ndx]) + " bytes: " + str(v))
        pkt = pkt + v
        for b in v:
            crc += int(b)

    # Need to indicate how many numbers, so I know how many format
    # indicators to read at the other end.
    pkt = int_to_bytes(len(fmt), 8) + pkt
    pkt = b"\x10" + pkt
    crc += len(fmt)
    pkt = pkt + int_to_bytes(crc % 128, 8)

    # print("Num Bytes: " + str(len(pkt)) + " crc: " + str(crc % 128) + " pkt: " + str(pkt))
    try_lock()
    try:
        i2c.writeto(ADDR, pkt, stop = True)
    finally:
        i2c.unlock()

def i2c_cmd(the_bytes):
    try_lock()
    try:
        i2c.writeto(ADDR, bytes(the_bytes), stop = True)
    finally:
        i2c.unlock()

def flush_cache():
    i2c_cmd([0x30])

def reset_sequence_number():
    i2c_cmd([0x3F])

def erase_cache():
    i2c_cmd([0x31])

def scan_bus():
    try_lock()
    try:
        print("I2C addresses found:", [hex(device_address) for device_address in i2c.scan()])
    finally:
        i2c.unlock()
    time.sleep(1)

# scan_bus()
erase_cache()
reset_sequence_number()

count = 0

def minutes(per_sec, num_min):
    return per_sec * 60 * num_min

per_sec = 10
total_min = 60 * 6

while count < minutes(per_sec, total_min):
    count = count + 1
    pkt = [count,
           # YY, MM, DD,
           19, random.randint(0, 11), random.randint(0, 30),
           # HH, mm, ss
           random.randint(0, 24), random.randint(0, 59), random.randint(0, 60),
           # Temperature will be two bytes
           random.randint(0, 255), random.randint(0, 255),
           # Accelerometer will be assumed to be three, 12-bit readings.
           # This is a worst-case scenario.
           random.randint(0, 4096), random.randint(0, 4096), random.randint(0, 4096)
           ]
    # print(pkt)
    # https://learn.adafruit.com/arduino-to-circuitpython/time
    # Seems to be taking around 20-25ms. The variability is because, sometimes,
    # there is a pause to flush to the SD card. The cache is small.
    start = time.monotonic()
    data_packet(pkt)
    end = time.monotonic()
    # print("T: " + str((end - start) * 1000))

    neo[0] = (random.randint(64, 127), random.randint(64, 127), random.randint(64, 127))
    time.sleep((per_sec / 100.0) - (end - start))

flush_cache()
neo[0] = (127, 0, 80)
print("DONE")