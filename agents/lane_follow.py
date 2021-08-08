from Planning.Frenet import F_Path
from Planning.cubic_spline import Spline2D
from Tracking.mpc_util import Vehicle_model, State
from Tracking.mpc_controller import MPC_controller
import carla
import matplotlib.pyplot as plt
import numpy as np

class lane_follow_agent(object):
    def __init__(self, player) -> None:
        super().__init__()
        self.physics_control = player.get_physics_control()
        self.max_speed = player.get_speed_limit()
        self.max_steer_angle = max([wheel.max_steer_angle for wheel in self.physics_control.wheels])
        print("max steer angle", self.max_steer_angle/180*np.math.pi)
        model = Vehicle_model(4.5, 1.8, np.deg2rad(self.max_steer_angle), self.max_speed, -self.max_speed, 2.5, 100)
        self.controller =  MPC_controller(model, DT=1/10, T=1/2)
        self.v = 0
    
    def take_action(self, state):
        self.generate_baseline(state)
        # self.show_state(state)
        # waypoints = state['waypoints']
        # point_lists = []
        # for path in waypoints:
        #     point_lists.append(list(map(lambda a:(a[0].transform.location.x, a.transform.location.y), path)))
        target_dir = self.baseline.calc_yaw(self.baseline.s[1]) /np.math.pi * 180
        self.yaw = state['mystate'].rotation.yaw / 180 * np.math.pi
        print("yaw: ",self.yaw)
        self.x = state['mystate'].location.x
        self.y = state['mystate'].location.y
        self.v = np.math.sqrt(state['myvelocity'].x*state['myvelocity'].x+state['myvelocity'].y*state['myvelocity'].y)
        state = State(self.x, self.y, self.yaw, self.v)
        # t, x, y, yaw, v, d, a = self.controller.tracking(state, self.baseline)
        angle = self.baseline.calc_yaw(self.baseline.s[0]) - self.yaw
        control = carla.VehicleControl()
        control.steer = np.clip(angle/np.math.pi*180/self.max_steer_angle, -1.0, 1.0)
        control.throttle = np.clip(0.55-abs(control.steer)/2, -1, 1)
        control.hand_brake = False
        control.manual_gear_shift = False
        print("angle: ", angle)
        # print("v: ", v[-1], "maxv: ", self.max_speed, " dir: ", d[-1])
        return control
    
    def generate_baseline(self, state):
        waypoints = state['waypoints']
        self.baseline_x = [state['mystate'].location.x]
        self.baseline_y = [state['mystate'].location.y]
        for points in waypoints[0]:
            self.baseline_x.append(points[0].transform.location.x)
            self.baseline_y.append(points[0].transform.location.y)
        self.baseline = Spline2D(self.baseline_x, self.baseline_y)
    
    def show_state(self, state):
        waypoints = state['waypoints']
        points_x = []
        points_y = []
        for path in waypoints:
            points_x.extend(list(map(lambda a:a[0].transform.location.x, path)))
            points_y.extend(list(map(lambda a:a[0].transform.location.y, path)))
        plt.plot(self.baseline_x, self.baseline_y, '-r',label='baseline')
        plt.plot(points_x, points_y, 'bo',label='waypoints')
        plt.plot([state['mystate'].location.x],[state['mystate'].location.y],'go',label='mypos')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.show()
