import numpy as np

from Sandbox import Sandbox
from common import dt, plt_show_maxed


class GTurn(Sandbox):
    def __init__(self, twr, mass, mflow, thrust_angle=5):
        self.T = mass*self.G*twr
        self.M = mass
        self.mflow = mflow
        self.a = np.deg2rad(thrust_angle)

    def simulate(self, start_vel, start_angle, end_angle):
        start_angle = np.deg2rad(start_angle)
        end_angle = np.deg2rad(end_angle)
        x = [0]
        y = [0]
        vx = [start_vel * np.cos(start_angle)]
        vy = [start_vel * np.sin(start_angle)]
        a = [start_angle]
        t = [0]
        m = self.M
        while a[-1] > end_angle:
            accel = self.T / m
            thrust_angle = a[-1] - self.a
            ax = accel * np.cos(thrust_angle)
            ay = accel * np.sin(thrust_angle) - self.G
            m -= self.mflow * dt
            vx.append(vx[-1] + ax * dt)
            vy.append(vy[-1] + ay * dt)
            x.append(x[-1] + vx[-1] * dt)
            y.append(y[-1] + vy[-1] * dt)
            a.append(np.arctan2(vy[-1], vx[-1]))
            t.append(t[-1] + dt)
            print(t[-1], vx[-1], vy[-1], np.sqrt(vx[-1] * vx[-1] + vy[-1] * vy[-1]))
        return t, a, x, y, vx, vy


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    sim = GTurn(2, 15, 0.01, 5)
    t, a, x, y, vx, vy = sim.simulate(50, 90, 30)
    a = np.rad2deg(a)
    plt.plot(t, a)
    plt.quiver(t, a, vx, vy, units='xy', width=0.02, angles='xy')
    plt_show_maxed()
