from .mpc_util import Vehicle_model, State
from Planning.cubic_spline import Spline2D
from Planning.Frenet import Cartesian_to_Frenet
import numpy as np
import math
import cvxpy
import matplotlib.pyplot as plt

class MPC_controller(object):
    """
    model predict control for tracking
    """
    def __init__(self, vehicle_model, target_speed=0.5, NX=4, NU=2, DT=0.2, T=1) -> None:
        """
        args:
            NX: number of state
            NU: number of input
            vehicle_model: vehicle kinematic model
            DT: time interupt
        """
        super().__init__()
        self.NX = NX
        self.NU = NU
        self.DT = DT
        self.T = T
        self.target_speed = vehicle_model.MAX_SPEED/2.0
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix
        self.model = vehicle_model
    
    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """
        times = int(self.T / self.DT)
        x = cvxpy.Variable((self.NX, times + 1))
        u = cvxpy.Variable((self.NU, times))

        cost = 0.0
        constraints = []

        for t in range(0, times):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.model.linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t], self.DT)
            constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.model.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, times] - x[:, times], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.model.MAX_SPEED]
        constraints += [x[2, :] >= self.model.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.model.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.model.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.model.get_nparray_from_matrix(x.value[0, :])
            oy = self.model.get_nparray_from_matrix(x.value[1, :])
            ov = self.model.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.model.get_nparray_from_matrix(x.value[3, :])
            oa = self.model.get_nparray_from_matrix(u.value[0, :])
            odelta = self.model.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov
    
    def calc_ref_trajectory(self, state, spline):
        times = int(self.T/self.DT)
        xref = np.zeros((self.NX, times + 1))
        dref = np.zeros((1, times + 1))
        snow, l, x, y = Cartesian_to_Frenet(state.x, state.y, spline)
        
        xref[0, 0] = x
        xref[1, 0] = y
        xref[2, 0] = self.target_speed
        xref[3, 0] = spline.calc_yaw(snow)
        dref[0, 0] = 0.0

        travel = snow
        for i in range(1,times+1):
            snow += self.target_speed * self.DT
            if snow > spline.s[-1]:
                snow = spline.s[-1]
            x, y = spline.calc_position(snow)
            yaw = spline.calc_yaw(snow)
            xref[0, i] = x
            xref[1, i] = y
            xref[2, i] = self.target_speed
            xref[3, i] = yaw
            dref[0, i] = 0.0
        
        return xref, dref
    
    def predict_motion(self, x0, oa, od, xref):
        T = int(self.T / self.DT)
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, T + 1)):
            state = self.model.update_state(state, ai, di, self.DT)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar
    
    def tracking(self, initial_state, spline, show_animation=True):
        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]

        """
        goal = spline.calc_position(spline.s[-1])
        state = initial_state

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]

        times = int(self.T / self.DT)
        odelta, oa = [0.0]* times, [0.0]* times

        while time < self.T:
            xref, dref = self.calc_ref_trajectory(state, spline)
            self.model.update_state(state, oa[-1], odelta[-1], self.DT)
            x0 = [state.x, state.y, state.v, state.yaw]  # current state
            xbar = self.predict_motion(x0, oa, odelta, xref)
            

            oa, odelta, ox, oy, oyaw, ov = self.linear_mpc_control(
                xref, xbar, x0, dref)

            if odelta is not None:
                di, ai = odelta[0], oa[0]

            state = self.model.update_state(state, ai, di, self.DT)
            time = time + self.DT

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            d.append(di)
            a.append(ai)

            dx = state.x - goal[0]
            dy = state.y - goal[1]
            if math.hypot(dx, dy) < 0.01:
                print("Goal")
                break

            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                if ox is not None:
                    plt.plot(ox, oy, "xr", label="MPC")
                spline.show()
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
                self.model.show(state.x, state.y, state.yaw, steer=di)
                plt.axis("equal")
                plt.grid(True)
                plt.title("Time[s]:" + str(round(time, 2))
                        + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
                plt.pause(0.0001)

        return t, x, y, yaw, v, d, a

if __name__ == "__main__":
    model = Vehicle_model(0.45, 0.2, np.deg2rad(45.0), 55.0/3.6, -20.0/3.6, 0.25, 1.0)
    MPC = MPC_controller(model)
    x = [-2.5, 0.0, 2.5, 5.0, 7.5]
    y = [0.7, -3.0, 5.0, 6.5, 0.0]
    sp = Spline2D(x, y)
    initial_state = State()
    MPC.tracking(initial_state, sp)
