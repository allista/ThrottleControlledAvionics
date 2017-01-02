import sys
import numpy as np
import matplotlib.pyplot as plt


dt = 0.02


def clampL(x, L): return x if x > L else L


def clampH(x, H): return x if x < H else H


def clamp(x, L, H):
    if x > H: return H
    elif x < L: return L
    return x


def clamp01(x):
    if x > 1: return 1.0
    elif x < 0: return 0.0
    return x


def lerp(f, t, time): return f+(t-f)*clamp01(time)


def center_deg(a):
    a %= 360
    if a > 180: a -= 360
    return a


class PID(object):
    def __init__(self, p, i, d, min_a, max_a):
        self.P = float(p)
        self.I = float(i)
        self.D = float(d)
        self.min = float(min_a)
        self.max = float(max_a)
        self.reset()

    def setPID(self, p, i, d):
        self.P = p
        self.I = i
        self.D = d
        self.reset()

    def setFrom(self, pid):
        self.P = pid.P
        self.I = pid.I
        self.D = pid.D
        self.reset()

    def pack(self):
        return [self.P, self.I, self.D]

    def reset(self):
        self.value = 0
        self.ierror = 0
        self.perror = 0
        self.action = 0

    def update(self, err):
        if self.perror == 0: self.perror = err
        if self.ierror*err < 0: self.ierror = 0
        old_ierror = self.ierror
        self.ierror += err*dt
        act = self.P * err + self.I * self.ierror + self.D * (err - self.perror) / dt
        clamped = clamp(act, self.min, self.max)
        if clamped != act: self.ierror = old_ierror
        self.perror = err
        self.action = clamped
        return self.action

    def sim(self, PV, SV, T):
        V = [0.0]
        for sv in SV[1:]:
            act = self.update(sv-V[-1])
            if np.isinf(act) or np.isnan(act):
                act = sys.float_info.max
            V.append(V[-1]+act)
        #plot
        V = np.array(V); SV = np.array(SV)
        plt.subplot(3,1,1)
        plt.plot(T, SV)
        plt.subplot(3,1,2)
        plt.plot(T, V)
        plt.subplot(3,1,3)
        plt.plot(T, V-SV)

    def __str__(self, *args, **kwargs):
        return ('p % 8.5f, i % 8.5f, d % 8.5f, min % 8.5f, max % 8.5f; action = % 8.5f'
                % (self.P, self.I, self.D, self.min, self.max, self.action))


class PID2(PID):
    def update(self, err):
        if self.perror == 0: self.perror = err
        d = self.D * (err - self.perror) / dt
        self.ierror = clamp(self.ierror + self.I * err * dt if abs(d) < 0.6 * self.max else 0.9 * self.ierror, self.min, self.max)
        self.perror = err
        self.action = clamp(self.P * err + self.ierror + d, self.min, self.max)
#         print '%f %+f %+f' % (self.P*err, self.ierror, d)
        return self.action
