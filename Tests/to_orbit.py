
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math

class Vessel(object):
    R  = 600000
    G  = 9.81
    cG = G*R*R

    @classmethod
    def StG(cls, h):
        return cls.cG/((cls.R+h)**2)

    def __init__(self, M, T, mflow, Cd=1.0, S=2.0):
        self.M = M
        self.T = T
        self.Cd = Cd
        self.S = S
        self.mflow = mflow
        self._T2mflow = self.T/self.mflow

    def full_thrust_velocity(self, dt):
        return self._T2mflow*math.log(self.M/(self.M-self.mflow*dt))-self.G*dt

    def full_thrust_height(self, dt):
        m1 = self.M-self.mflow*dt
        return self._T2mflow*(m1/self.mflow*math.log(m1/self.M)+dt)-self.G*dt*dt/2

    def full_thrust_ApA(self, dt):
        return self.full_thrust_velocity(dt)**2/2/self.G + self.full_thrust_height(dt)

    def ttb_for_ApA(self, ApA, tol=0.01):
        t0 = 0.0
        t1 = 1.0
        while self.full_thrust_ApA(t1) < ApA: t1 *= 2
        while(t1-t0 > tol):
            t = (t0+t1)/2.0
            apa = self.full_thrust_ApA(t)
            if apa > ApA: t1 = t
            else: t0 = t
        return (t0+t1)/2.0

    def tta(self, ApA, tol=0.01):
        ttb = self.ttb_for_ApA(ApA, tol)
        return self.full_thrust_velocity(ttb)/self.G+ttb

    def m1(self, dt):
        return self.M - self.mflow * dt

    def h(self, t, ttb):
        if t < ttb: return self.full_thrust_height(t)
        t2 = t-ttb
        return self.full_thrust_height(ttb)+self.full_thrust_velocity(ttb)*t2-self.G*t2*t2/2

    def v(self, t, ttb):
        if t < ttb: return self.full_thrust_velocity(t)
        t2 = t - ttb
        return self.full_thrust_velocity(ttb) - self.G * t2

    @staticmethod
    def angle(t, ttb):
        if t >= ttb: return 0
        return np.pi/2*(ttb-t)/ttb

    def ApA_stats(self, ApA):
        ttb = self.ttb_for_ApA(ApA)
        print 'apoapsis:     %.1f km'  % (self.full_thrust_ApA(ttb) / 1000)
        print 'TTB:          %.1f s'   % ttb
        print 'burn ends at: %.1f km'  % (self.full_thrust_height(ttb) / 1000)
        print 'V max:        %.1f m/s' % self.full_thrust_velocity(ttb)
        print 'TTA:          %.1f s'   % self.tta(ApA)
        print 'End Mass:     %.1f t'   % self.m1(ttb)

    def __str__(self):
        return 'M: %.1f t, T: %.1f kN, mFlow: %f t/s' % (self.M, self.T, self.mflow)

    def freefall(self, m, h, v, atm, dt=0.01):
        H = h
        while v > 0:
            H += v*dt
            v -= self.StG(H)*dt
            if atm: v -= (atm(H) * (v**2) * self.Cd * self.S) / 2 / m * dt
        return H

    def simulate(self, ApA, dt=0.01, atm=None):
        t = [0.0]
        v = [0.0]
        h = [0.0]
        Tf = [1.0]
        m = self.M
        thrust = True
        hmove = ApA
        while v[-1] >= 0:
            if thrust:
                dm = self.mflow*dt
                if m <= dm:
                    thrust = False
                    continue
                m -= dm
                apa = self.freefall(m, h[-1], v[-1], atm, dt * 4)
                dapa = ApA-apa
                vv = dapa/max(v[-1],1)
                hv = hmove/max(dapa, 60)*60*math.sqrt(min(max(h[-1]/70000, 0), 1))*min((apa-h[-1])/100, 1)
                hmove -= hv*dt
                Tf.append(math.sin(math.atan2(vv, hv)))
                T = self.T*Tf[-1]
                v1 = v[-1]+(T/m-self.StG(h[-1]))*dt
                thrust = ApA-apa > 1
            else:
                v1 = v[-1] - self.StG(h[-1]) * dt
                Tf.append(0)
            if atm:
                v1 -= (atm(h[-1]) * (v[-1] ** 2) * self.Cd * self.S) / 2 / m * dt
            v.append(v1)
            h.append(h[-1]+v1*dt)
            t.append(t[-1]+dt)
        return t, h, v, Tf

if __name__ == '__main__':
    # TCA Test 6.RendezvouAutopilot: T 928.8749*0.919181502342862, M 67.045, mflow 0.2814892
    dt = 0.5
    ApA = 100000

    vsl = Vessel(67.045, 928.8749*0.9, 0.2814892, 0.0006, 44.19966)

    # ttb = vsl.ttb_for_ApA(ApA)
    # time = np.arange(0, vsl.tta(ApA)+dt, dt)
    # h = np.vectorize(lambda t: vsl.h(t, ttb))(time)
    # v = np.vectorize(lambda t: vsl.v(t, ttb))(time)
    # plt.subplot(311)
    # plt.plot(time, h)
    # plt.subplot(312)
    # plt.plot(time, v)

    L = 0.0065
    P0 = 101325
    T0 = 298
    M = 0.029
    R = 8.31447
    def density(h):
        T = T0-L*h
        if T < 0: return 0
        P = P0*((1-L*h/T0)**(9.81*M/R/L))
        return P*M/R/T

    t, h, v, Tf = vsl.simulate(ApA, dt, atm=density)
    plt.subplot(311)
    plt.plot(t, h)
    plt.subplot(312)
    plt.plot(t, v)
    plt.subplot(313)
    plt.plot(t, Tf)

    df = pd.read_csv('REN.csv', names=['UT', 'ApA', 'v', 'angle'])
    df.UT -= df.UT[0]
    df = df.loc[df.UT > 0,:]
    df.Tf = np.sin(df.angle/180*np.pi)
    plt.subplot(311)
    plt.plot(df.UT, df.ApA)
    plt.subplot(312)
    plt.plot(df.UT, df.v)
    plt.subplot(313)
    plt.plot(df.UT, df.Tf)

    plt.show()
