import json
import logging
import socket
import threading
import math

class VehicleData:
    def __init__(self):
        self.name = ""
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.speed = 0


class UDPClient:
    def __init__(self, ip, port, send_port, vehicle_name):
        self.send_port = send_port
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.vehicle_data = VehicleData()
        self.neighbor_vehicle_data = []
        self.vehicle_name = vehicle_name

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)  # 设置日志级别为 DEBUG
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)  # 添加控制台日志处理

    def start(self):
        threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        while True:
            data0, addr = self.sock.recvfrom(10240)
            data1 = data0.decode()

            data = json.loads(data1)
            try:
                for vehicle in data['vehicles']:
                    if vehicle['name'] == self.vehicle_name:
                        self.vehicle_data.name = vehicle['name']
                        self.vehicle_data.x = vehicle['x']
                        self.vehicle_data.y = vehicle['y']
                        self.vehicle_data.yaw = vehicle['yaw']
                        self.vehicle_data.speed = vehicle['speed']
                        self.neighbor_vehicle_data = []
                        for other_vehicle in data['vehicles']:
                            if other_vehicle['name'] != self.vehicle_name:
                                neighbor_vehicle = VehicleData()
                                neighbor_vehicle.name = other_vehicle['name']
                                neighbor_vehicle.x = other_vehicle['x']
                                neighbor_vehicle.y = other_vehicle['y']
                                neighbor_vehicle.yaw = other_vehicle['yaw']/ 180 * math.pi
                                neighbor_vehicle.speed = other_vehicle['speed'] 
                                self.neighbor_vehicle_data.append(neighbor_vehicle)
            except Exception as e:
                self.logger.error(e)

    def send(self, message):
        self.sock.sendto(message.encode(), (self.ip, self.send_port))
        self.logger.info("send message: " + message)

    def get_vehicle_state(self):
        return self.vehicle_data
    
    def get_neighbor_vehicle_state(self):
        return self.neighbor_vehicle_data

    def send_control_command(self, v, w):
        message = '{"name":"' + self.vehicle_name + '","vx":%f,"vz":%f}' % (v, w)
        self.send(message)
