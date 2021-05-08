import dronekit
import pymysql
import PathProgramming
import threading
import DynamicAvoid
from TaskAssignment import *
import serial

ser = serial.Serial("/dev/ttyUSB1",115200,timeout=0.4)
class GetHomeLoc:
    def __init__(self):
        self.get = False
        self.Copter0Home = [None,b'F']
        self.Copter1Home = [None,b'F']
        self.Copter2Home = [None,b'F']
    def start(self):
        threading.Thread(target=self.Receive,daemon=True).start()
        threading.Thread(target=self.Sent, daemon=True).start()
        while True:
            print("接收各機當前位置")
            if self.get:
                print("接收完成")
                break
            time.sleep(1)

    def Receive(self):
        while True:
            data = ser.readline()
            if data != b'':
                data_split = data.split(b',')
                if len(data_split) == 6:
                    if data_split[0] == b'0':
                        self.Copter0Home = [LocationGlobalRelative(float(data_split[1]), float(data_split[2]), float(data_split[3])),data_split[-2]]
                    elif data_split[0] == b'1':
                        self.Copter1Home = [LocationGlobalRelative(float(data_split[1]), float(data_split[2]), float(data_split[3])),data_split[-2]]
                    elif data_split[0] == b'2':
                        self.Copter2Home = [LocationGlobalRelative(float(data_split[1]), float(data_split[2]), float(data_split[3])),data_split[-2]]
                if self.Copter0Home[1] == b'T' and self.Copter1Home[1] == b'T' and self.Copter2Home[1] == b'T':
                    self.get = True
                    break
            time.sleep(0.4)

    def Sent(self):
        while True:
            if self.Copter0Home[0] != None and self.Copter1Home[0] != None and self.Copter2Home[0] != None:
                ser.write(b'0,' + str(Home.lat).encode('utf-8') + b','
                            + str(Home.lon).encode('utf-8') + b','
                            + str(Home.alt).encode('utf-8') + b','
                            + b'T,' + b'\r\n')
            else:
                ser.write(b'0,' + str(Home.lat).encode('utf-8') + b','
                          + str(Home.lon).encode('utf-8') + b','
                          + str(Home.alt).encode('utf-8') + b','
                          + b'F,' + b'\r\n')
            if self.get:
                break
            time.sleep(0.4)

class RunDSL:
    def __init__(self, NoFlyZones, UAVRadius, Path):  #給入禁航區清單,無人機半徑,初始路徑
        self._stop = False
        self.Searching = False

        self.Path = Path
        self.index = 1
        self.DSL = DynamicAvoid.DStarLite(Path[0], Path[1], NoFlyZones, UAVRadius)

    def WaitObsInsert(self):      #等待障礙物產生,此處為手動input障礙物座標
        while not self._stop:
            obs = [item.split(',') for item in input(" 障礙物經緯座標 : ").split()]
            obs_p1 = dronekit.LocationGlobalRelative(float(obs[0][0]), float(obs[0][1]))
            obs_p2 = dronekit.LocationGlobalRelative(float(obs[1][0]), float(obs[1][1]))

            self.DSL.Obstacle.append([obs_p1, obs_p2])
            print(self.DSL.Obstacle)
            time.sleep(0.3)

    def start(self):   #開始地圖搜索與感測障礙物
        threading.Thread(target=self.Search).start()
        threading.Thread(target=self.WaitObsInsert).start()

    def stop(self):
        self._stop = True

    def Search(self):
        while len(self.Path) != self.index:
            if self.DSL.ScanForObstacles():
                self.Searching = True
                self.DSL.ComputeShortestPath()
                if self.DSL.U != None:
                    self.Path = self.Path[:self.index] + self.DSL.U + self.Path[self.index:]
                    self.Searching = False

            else:
                pass

            self.DSL.Goal = self.Path[self.index]
            if self._stop:
                break

            time.sleep(0.2)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.98:
            print("Reached target altitude")
            break
        time.sleep(1)


db = pymysql.Connect(host='140.130.20.168',user='gpspm2.5',password='gpspm2.5',db='googlemap')
#資料表指令
paths_data = "SELECT * FROM googlemap.markers WHERE type = 0"
noFlyZones_data = "SELECT * FROM googlemap.markers WHERE type = 1"

cursor = db.cursor()

cursor.execute(paths_data)
results = cursor.fetchall()
# 0 type,1 number,2 id,3 lat,4 lon,5 alt

paths = [[] for i in range(max([result[1] for result in results])+1)]
for i in range(len(paths)):
    paths[i].extend([dronekit.LocationGlobalRelative(result[3],result[4],result[5]) for result in results if result[1] == i])

cursor.execute(noFlyZones_data)
results = cursor.fetchall()
# 0 type,1 number,2 id,3 lat,4 lon,5 alt

noFlyZones = [[] for i in range(max([result[1] for result in results])+1)]
for i in range(len(noFlyZones)):
    noFlyZones[i].extend([dronekit.LocationGlobalRelative(result[3], result[4], result[5]) for result in results if result[1] == i])

#無人載具連線設定
vehicle = connect('127.0.0.1:14550', wait_ready=True)   #載具連線
vehicle._master.param_set_send('WP_YAW_BEHAVIOR', 0)    #設定載具經過目標不轉yaw

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
Home = vehicle.home_location

#開始接收各機起飛位置
# receive = GetHomeLoc()
# receive.Copter0Home = (Home, b'T')
# receive.start()

arm_and_takeoff(16)

#分配各機飛行任務
#CopterHome = [receive.Copter0Home[0],receive.Copter1Home[0],receive.Copter2Home[0]]
CopterHome = [dronekit.LocationGlobalRelative(23.729723, 120.418376, 0),dronekit.LocationGlobalRelative(23.729753, 120.418406, 0),dronekit.LocationGlobalRelative(23.729783, 120.418436, 0)]
UAVRadiusList = [1,1,1]
AssignRoute = TaskAssignment.Search(CopterHome,[path[1] for path in paths],noFlyZones,UAVRadiusList)


Path = next((path for path in paths if path[1].lat == AssignRoute[0][-1].lat and path[1].lon == AssignRoute[0][-1].lon))
Path = AssignRoute[0]+Path[2:]
Path[-1] = Path[0]


DStarlite = RunDSL(noFlyZones,1,Path)    #初始化D*Lite
DStarlite.start()                   #開始更新地圖環境

while True:

    current_loc = vehicle.location.global_relative_frame
    target_loc = dronekit.LocationGlobalRelative(DStarlite.Path[DStarlite.index].lat,
                                        DStarlite.Path[DStarlite.index].lon, 16)

    DStarlite.DSL.Start = current_loc
    if not DStarlite.Searching:
        vehicle.simple_goto(target_loc, groundspeed = 2)
    else:
        send_ned_velocity(0, 0, 0)

    wp_dist = PathProgramming.CalculationFormula.CalculateDistance(current_loc.lat, current_loc.lon, current_loc.alt,
                                                                   target_loc.lat, target_loc.lon, target_loc.alt)
    if wp_dist < 2:
        DStarlite.index += 1
        DStarlite.DSL.initial()

    if DStarlite.index == len(DStarlite.Path):
        DStarlite.stop()
        vehicle.mode = VehicleMode("RTL")
    if vehicle.location.global_relative_frame.alt <= 0.5:
        break

    time.sleep(0.2)