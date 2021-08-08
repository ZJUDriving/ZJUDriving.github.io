import numpy as np
from .cubic_spline import Spline, Spline2D
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

def Cartesian_to_Frenet(tx, ty, baseline):
    nearst_x, nearst_y = baseline.calc_position(0)
    nearst_s = 0
    min_distance = np.hypot(tx-nearst_x, ty-nearst_y)
    for i in np.arange(0, baseline.s[-1], 0.1):
        px, py = baseline.calc_position(i)
        distance = np.hypot(tx-px, ty-py)
        if min_distance > distance:
            min_distance = distance
            nearst_x, nearst_y = px, py
            nearst_s = i

    def func(s):
        dx = baseline.sx.calcd(s)
        dy = baseline.sy.calcd(s)
        x, y = baseline.calc_position(s)
        return dx * tx + dy * ty - (dx*x + dy*y)

    nearst_s = fsolve(func, nearst_s)
    print(func(nearst_s))
    nearst_x, nearst_y = baseline.calc_position(nearst_s)
    l = np.hypot(tx - nearst_x, ty - nearst_y)
    return nearst_s, l, nearst_x, nearst_y

def Cartesian_to_Frenet_sample(tx, ty, baseline):
    nearst_x, nearst_y = baseline.calc_position(0)
    nearst_s = 0
    min_distance = np.hypot(tx-nearst_x, ty-nearst_y)
    for i in np.arange(0, baseline.s[-1], 0.05):
        px, py = baseline.calc_position(i)
        distance = np.hypot(tx-px, ty-py)
        if min_distance > distance:
            min_distance = distance
            nearst_x, nearst_y = px, py
            nearst_s = i
    return nearst_s, min_distance, nearst_x, nearst_y

class F_Path():
    def __init__(self, x, y, baseline) -> None:
        self.x = x
        self.y = y
        self.baseline = baseline
        self.s, self.l = Cartesian_to_Frenet(x, y, baseline)
        self.path = Spline(self.s, self.l)
        
    def generate_spline_path(self):
        pass
    
    def get_l(self, s):
        if s > self.s[-1]:
            print("further than plan")
            raise IndexError
        elif s < self.s[0]:
            print("already past")
            raise IndexError
    
if __name__ == "__main__":
    
    print("test Frenet")
    x = [-2.5, 0.0, 2.5, 5.0, 7.5]
    y = [0.7, -3.0, 5.0, 6.5, 0.0]
    sp = Spline2D(x, y)
    tx, ty = 0.0, 0.0
    s, l, dx, dy = Cartesian_to_Frenet(tx, ty, sp)
    s1, l1, dx1, dy1 = Cartesian_to_Frenet_sample(tx, ty, sp)
    rx, ry = [], []
    for i_s in np.arange(0, sp.s[-1], 0.1):
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
    plt.plot(rx, ry, "-r", label="path")
    plt.plot([tx], [ty], '-yo', label="origin")
    plt.plot([dx], [dy], '-bo', label="continue")
    plt.plot([dx1], [dy1], '-go', label="decrete")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    plt.show()

