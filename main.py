import json
import math
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001 
        self.server_ip = '192.168.1.100'

        net = "z2JkjyynJGQ6oCucSM9fhrEDYLsf,192.168.1.109,8700,8701"
        if net != "":
            net = net.split(",")
            self.vehicle_name = net[0]
            self.server_ip = net[1]
            self.udp_port = int(net[2])
            self.udp_send_port = int(net[3])

        print(self.vehicle_name)
        print(self.udp_port)
        print(self.udp_send_port)
        print(self.server_ip)
        self.udp_client = UDPClient(self.server_ip, self.udp_port, self.udp_send_port, self.vehicle_name)

        self.m_v = 0
        self.m_x = 0
        self.m_y = 0
        self.m_yaw = 0
        self.vehpos_initial_index = 0
        self.num_preview = 8
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = 10  # hz
        self.wheel_base = 2.7

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes/leftInside.json')
        while True:
            vehicle_data = self.udp_client.get_vehicle_state()
            self.m_x = vehicle_data.x
            self.m_y = vehicle_data.y
            self.m_yaw = vehicle_data.yaw / 180 * math.pi
            self.m_v = 10
            self.update_vehpos_index()
            self.search_target_pos()

            v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
            self.udp_client.send_control_command(v, w)

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def load_route(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            json_track = json.load(file)

        if isinstance(json_track, list):
            self.X_points = [point["x"] for point in json_track]
            self.Y_points = [point["y"] for point in json_track]
        elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
            self.X_points = json_track["X"]
            self.Y_points = json_track["Y"]
        else:
            raise ValueError(
                "Unsupported route format. Expected [{'x': ..., 'y': ...}, ...] "
                "or {'X': [...], 'Y': [...]}."
            )

        if len(self.X_points) != len(self.Y_points) or len(self.X_points) == 0:
            raise ValueError("Route file must contain the same non-zero number of X and Y points.")

        self.X_points = [float(x) for x in self.X_points]
        self.Y_points = [float(y) for y in self.Y_points]

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        ###################################
        ##输出控制：速度（v）和转向角（steering_angle）
        ##请在此处补全纯跟踪算法的核心计算公式
        ##所需参数：
        ## m_x, m_y          --车辆位置
        ## m_yaw             --车辆航向角
        ## target_pos        --目标点
        ## self.wheel_base   --轴距
        v, steering_angle=self.m_v, 0

        ###################################
        w = v * math.tan(steering_angle) / self.wheel_base
        return v, w

    def search_vehicle_initial_index(self):
        min_distance = float('inf')
        nearest_index = 0

        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        self.vehpos_initial_index = nearest_index

    
    def find_nearest_point_index(self, target_x, target_y):
        min_distance = float('inf')
        nearest_index = -1
    
        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((target_x - this_point_x) ** 2 + (target_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index
    
    def update_vehpos_index(self):
        min_distance = float('inf')
        nearest_index = 0
        for i in range(40):
            find_index = (self.vehpos_initial_index + i) % len(self.X_points)
            this_point_x = self.X_points[find_index]
            this_point_y = self.Y_points[find_index]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = find_index
        if min_distance > 25:
            self.search_vehicle_initial_index()
        else:
            self.vehpos_initial_index = nearest_index

    def search_target_pos(self):
        target_x = self.m_x + self.num_preview * math.cos(self.m_yaw)
        target_y = self.m_y + self.num_preview * math.sin(self.m_yaw)
        target_pos_index = self.find_nearest_point_index(target_x, target_y)
        self.targetPos_Info[0] = self.X_points[target_pos_index]
        self.targetPos_Info[1] = self.Y_points[target_pos_index]

if __name__ == '__main__':
    control = Control()
    control.udp_client.start()
    control.control_node()
