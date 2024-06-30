import sys
import time
from datetime import datetime

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
import csv
import json
from numpy import arctan2, arcsin
import cflib
import time
import numpy as np
import struct
import os

NR_OF_DRONES = int(sys.argv[1])

folders = [f for f in os.listdir("logs/") if os.path.isdir(os.path.join("logs/", f))]
folder_idx = len(folders)
while os.path.exists("logs/log" + str(folder_idx) + "/"):
    folder_idx += 1
fileName = "logs/log" + str(folder_idx) + "/"
os.makedirs(fileName, exist_ok=True)

URI0 = 'radio://0/100/2M/E7E7E7E7EA'
URI1 = 'radio://0/100/2M/E7E7E7E7EB'
URI2 = 'radio://0/100/2M/E7E7E7E7EC'
URI3 = 'radio://0/100/2M/E7E7E7E7ED'
uri_list = [URI0, URI1, URI2, URI3]

uris = {URI0}
for i in range(1, NR_OF_DRONES):
    uris.add(uri_list[i])

positions = np.array([
    [0.00, 0.0, 0.0],
    [-1.95, 0.0, 0.0],
    [-3.90, 0.0, 0.0],
    [-11.5, 0.0, 3.14]])

start_mission = 1

class Logger:
    def __init__(self, drone_label):
        self.data_buf = []
        self.drone_label = drone_label

    def log(self, data, file):
        if data[0] == 0 or data[0] == 128 + 0:
            if data[1] == 0:
                self.data_buf = data[2:]
            else:
                self.data_buf.extend(data[2:])

            if (data[0] > 127):
                global start_mission
                if start_mission:
                    start_mission = 0
                    print("Time ", round(1000 * time.time(), 1))

                [id] = struct.unpack("<hxx", bytes(self.data_buf[:4]))
                [timestamp] = struct.unpack("<l", bytes(self.data_buf[4:8]))
                [x, y, yaw] = struct.unpack("<fff", bytes(self.data_buf[8:20]))
                tof = np.zeros((4, 8))
                for i in range(4):
                    for j in range(8):
                        from_idx = 20 + 8 * 2 * i + 2 * j
                        to_idx = 20 + 8 * 2 * i + 2 * (j+1)
                        [tof[i, j]] = struct.unpack("<h", bytes(self.data_buf[from_idx:to_idx]))

                pose = np.zeros(37)
                pose[0] = id
                pose[1] = timestamp
                pose[2] = x
                pose[3] = y
                pose[4] = yaw
                pose[5:] = tof.reshape(-1)
                
                with open(fileName + str(self.drone_label) + ".csv", 'a') as csvfile:
                    writer = csv.writer(csvfile,delimiter=',')
                    pose = [round(float(entry), 3) if i > 1 and i < 5 else int(entry) for i, entry in enumerate(pose)]

                    writer.writerow(pose)
                csvfile.close()

        if data[0] == 1 or data[0] == 128 + 1:
            [scan_id] = struct.unpack("<h", bytes(data[2:4]))

            with open(fileName + str(self.drone_label) + "s.csv", 'a') as csvfile:
                writer = csv.writer(csvfile,delimiter=',')
                writer.writerow([scan_id])
                csvfile.close()


def new_packet_rcv_0(packet):
    data = packet._get_data_l()
    log0.log(data, "A")

def new_packet_rcv_1(packet):
    data = packet._get_data_l()
    log1.log(data, "B")

def new_packet_rcv_2(packet):
    data = packet._get_data_l()
    log2.log(data, "C")

def new_packet_rcv_3(packet):
    data = packet._get_data_l()
    log3.log(data, "D")


def console_callback_0(text: str):
    print("A: ",text, end='')

def console_callback_1(text: str):
    print("B: ",text, end='')
    
def console_callback_2(text: str):
    print("C: ",text, end='')
    
def console_callback_3(text: str):
    print("D: ",text, end='')

def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

    if scf.cf.link_uri == "radio://0/100/2M/E7E7E7E7EA":
        scf.cf.console.receivedChar.add_callback(console_callback_0)
        scf.cf.add_port_callback(1, new_packet_rcv_0)

    if scf.cf.link_uri == "radio://0/100/2M/E7E7E7E7EB":
        scf.cf.console.receivedChar.add_callback(console_callback_1)
        scf.cf.add_port_callback(1, new_packet_rcv_1)

    if scf.cf.link_uri == "radio://0/100/2M/E7E7E7E7EC":
        scf.cf.console.receivedChar.add_callback(console_callback_2)
        scf.cf.add_port_callback(1, new_packet_rcv_2)

    if scf.cf.link_uri == "radio://0/100/2M/E7E7E7E7ED":
        scf.cf.console.receivedChar.add_callback(console_callback_3)
        scf.cf.add_port_callback(1, new_packet_rcv_3)

def set_param(scf):
    drone_id = ord(scf.cf.link_uri[-1]) - ord("A")
    print("Setting ", positions[drone_id][0], positions[drone_id][1], positions[drone_id][2])
    scf.cf.param.set_value('initial.xinit', '{:f}'.format(positions[drone_id][0]))
    scf.cf.param.set_value('initial.yinit', '{:f}'.format(positions[drone_id][1]))
    scf.cf.param.set_value('initial.yawinit', '{:f}'.format(positions[drone_id][2]))
    scf.cf.param.set_value('cmds.ready', '{:d}'.format(1))

def reset_param(scf):
    scf.cf.param.set_value('cmds.ready', '{:d}'.format(0))

log0 = Logger("A")
log1 = Logger("B")
log2 = Logger("C")
log3 = Logger("D")

cflib.crtp.init_drivers(enable_debug_driver=False)
factory = CachedCfFactory(rw_cache='./cache')
with Swarm(uris, factory=factory) as swarm:
    swarm.parallel(wait_for_param_download)
    swarm.parallel(set_param)
    
    while(1):
        time.sleep(1)
    
