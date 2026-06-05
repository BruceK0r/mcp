import json
import logging
import os
import socket
import threading
import math
import time

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
        self.packet_count = 0
        self.last_packet_time = 0.0
        self.last_seen_vehicle_names = []
        self.log_send_messages = os.environ.get("UDP_LOG_SEND", "0").lower() in ("1", "true", "yes", "on")

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG if self.log_send_messages else logging.INFO)
        self.logger.propagate = False
        if not self.logger.handlers:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

    def start(self):
        threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        while True:
            try:
                data0, addr = self.sock.recvfrom(10240)
                data1 = data0.decode()
                data = json.loads(data1)
                vehicles = data.get('vehicles', [])
                if not isinstance(vehicles, list):
                    vehicles = []
                self.packet_count += 1
                self.last_packet_time = time.time()
                self.last_seen_vehicle_names = [
                    vehicle.get('name', '') for vehicle in vehicles if isinstance(vehicle, dict)
                ]

                for vehicle in vehicles:
                    if not isinstance(vehicle, dict):
                        continue

                    if vehicle.get('name', '') == self.vehicle_name:
                        self.vehicle_data.name = vehicle.get('name', '')
                        self.vehicle_data.x = vehicle.get('x', self.vehicle_data.x)
                        self.vehicle_data.y = vehicle.get('y', self.vehicle_data.y)
                        self.vehicle_data.yaw = vehicle.get('yaw', self.vehicle_data.yaw)
                        self.vehicle_data.speed = vehicle.get('speed', self.vehicle_data.speed)
                        self.neighbor_vehicle_data = []
                        for other_vehicle in vehicles:
                            if not isinstance(other_vehicle, dict):
                                continue

                            if other_vehicle.get('name', '') != self.vehicle_name:
                                neighbor_vehicle = VehicleData()
                                neighbor_vehicle.name = other_vehicle.get('name', '')
                                neighbor_vehicle.x = other_vehicle.get('x', 0)
                                neighbor_vehicle.y = other_vehicle.get('y', 0)
                                neighbor_vehicle.yaw = other_vehicle.get('yaw', 0) / 180 * math.pi
                                neighbor_vehicle.speed = other_vehicle.get('speed', 0)
                                self.neighbor_vehicle_data.append(neighbor_vehicle)
            except Exception as e:
                self.logger.error(e)

    def send(self, message):
        try:
            self.sock.sendto(message.encode(), (self.ip, self.send_port))
        except OSError as exc:
            self.logger.error("send failed: %s", exc)
            return

        if self.log_send_messages:
            self.logger.debug("send message: %s", message)

    def get_vehicle_state(self):
        return self.vehicle_data
    
    def get_neighbor_vehicle_state(self):
        return self.neighbor_vehicle_data

    def get_debug_status(self):
        return {
            "packet_count": self.packet_count,
            "last_packet_time": self.last_packet_time,
            "last_seen_vehicle_names": list(self.last_seen_vehicle_names),
        }

    def send_control_command(self, v, w):
        message = '{"name":"' + self.vehicle_name + '","vx":%f,"vz":%f}' % (v, w)
        self.send(message)
