#!/usr/bin/env python

import socket
import struct


UDP_IP = "10.42.0.30"
UDP_PORT = 65432

navsat_fix_format = "2f"  # lat, lon

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

print("Ready")

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    msg = struct.unpack(navsat_fix_format, data)
    print("received: lat=%f  lon=%f" % (msg[0], msg[1]))
    # IMPORTANT: lat/lon coordinates with the value 999 should be treated as NaN