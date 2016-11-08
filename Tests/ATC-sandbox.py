import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

from itertools import combinations_with_replacement
from KSPUtils import Part, SearchTerm
from BioUtils.Tools.Multiprocessing import MPMain, parallelize_work

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


class Engine(object):
    def __init__(self, maxThrust, acceleration=0, deceleration=0):
        self.name = 'Engine'
        self.maxThrust = maxThrust
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.limit = 1
        self.lever = 1
        self.thrust = 0
        self.torque = 0

    def __str__(self):
        s = [self.name, 'maxThrust: %f' % self.maxThrust]
        if self.acceleration > 0:
            s.append('accelerationSpeed: %f' % self.acceleration)
        if self.deceleration > 0:
            s.append('decelerationSpeed: %f' % self.deceleration)
        return '\n'.join(s)

    def update(self):
        request = self.maxThrust*self.limit
        delta = request-self.thrust
        if abs(delta) < 0.000001:
            self.thrust = request
            return
        if delta > 0:
            if self.acceleration > 0:
                self.thrust = lerp(self.thrust, request, self.acceleration * dt)
            else: self.thrust = request
        elif delta < 0:
            if self.deceleration > 0:
                self.thrust = lerp(self.thrust, request, self.deceleration * dt)
            else: self.thrust = request
        self.torque = self.thrust*self.lever

    module_term = SearchTerm('PART/MODULE:ModuleEngines.*/')

    def clone(self):
        e = Engine(self.maxThrust, self.acceleration, self.deceleration)
        e.limit = self.limit
        e.thrust = self.thrust
        e.lever = self.lever
        e.torque = self.torque
        return e

    @classmethod
    def from_file(cls, path):
        p = Part.LoadFromFile(path)
        if p is None: return None
        p = list(p)[0]
        engine = cls.module_term.select(p)
        if not engine: return None
        engine = engine[0]
        try:
            maxThrust = float(engine.GetValue('maxThrust'))
            useResponse = bool(engine.GetValue('useEngineResponseTime'))
            accelSpeed = 0
            decelSpeed = 0
            if useResponse:
                accelSpeed = float(engine.GetValue('engineAccelerationSpeed'))
                decelSpeed = float(engine.GetValue('engineDecelerationSpeed'))
        except Exception as e:
            print str(e)
            return None
        e = Engine(maxThrust, accelSpeed, decelSpeed)
        e.name = p.name
        return e


class ATC(object):
    twoPi = np.pi * 2

    def __init__(self, engine, lever, MoI, atPID, avPID, base_level):
        self.error = 0
        self.atPID = atPID
        self.avPID = avPID
        self.base_level = base_level
        self.engineF = engine.clone()
        self.engineF.lever = lever
        self.engineF.maxThrust *= 2
        self.engineF.limit = base_level
        self.engineF.thrust = self.engineF.maxThrust*base_level
        self.engineR = self.engineF.clone()
        self.MoI = MoI
        self.MaxAA = self.engineF.maxThrust*self.engineF.lever/self.MoI
        self.AV = 0
        self.AA = 0

    def reset(self):
        self.AV = 0
        self.AA = 0
        self.engineF.limit = self.base_level
        self.engineF.thrust = self.engineF.maxThrust * self.base_level
        self.engineR.limit = self.base_level
        self.engineR.thrust = self.engineF.maxThrust * self.base_level

    def updateAV(self, av_error):
        self.avPID.update(av_error)
        if self.avPID.action > 0:
            self.engineF.limit = min(self.base_level + self.avPID.action, 1)
            self.engineR.limit = max(self.base_level - self.avPID.action, 0)
        else:
            self.engineR.limit = min(self.base_level - self.avPID.action, 1)
            self.engineF.limit = max(self.base_level + self.avPID.action, 0)
        self.engineF.update()
        self.engineR.update()
        self.AA = (self.engineF.torque - self.engineR.torque) / self.MoI
        self.AV += self.AA * dt

    def update(self):
        self.atPID.update(self.error)
        self.updateAV(self.atPID.action - self.AV)
        self.error -= self.AV*dt
        self.error %= self.twoPi
        if self.error < 0: self.error += self.twoPi
        if self.error > np.pi: self.error -= self.twoPi

    class ZeroStats(object):
        def __init__(self, time, speed, desc, units):
            self.time = time
            self.speed = speed
            self.desc = desc or 'Speed'
            self.units = units
            self.metric = self.time+self.speed*50

        def __str__(self):
            return ('Zero at: %f s\n'
                    '%s: %f %s' % (self.time, self.desc, self.speed, self.units))

    def simulate_constant_angular_velocity(self, needed_av, end_time, end_on_zero=False):
        t = 0
        time = [0]
        error = [needed_av]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while t < end_time:
            t += dt
            av_error = needed_av-self.AV
            self.updateAV(av_error)
            if not zero_stats and (av_error < self.tenth_deg or av_error*error[-1] < 0):
                zero_stats = self.ZeroStats(t, abs(self.AA), 'AA', 'rad/s2')
            error.append(av_error)
            thrust.append(self.engineF.thrust / self.engineF.maxThrust)
            time.append(t)
            if zero_stats and end_on_zero: break
        return time, np.fromiter(error, float)/np.pi*180, thrust, zero_stats

    tenth_deg = 0.1/180*np.pi

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        t = 0
        time = [0]
        error = [self.error]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while t < end_time:
            t += dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(t, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            thrust.append(self.engineF.thrust / self.engineF.maxThrust)
            time.append(t)
        return time, np.fromiter(error, float)/np.pi*180, thrust, zero_stats

    def optimize_av_PID(self, needed_av, aa_threshold, step, end_time, P, I, D):
        best_pid = None
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                for d in np.linspace(D[0], D[1], step):
                    self.avPID.setPID(p,i,d)
                    time, error, thrust, zero_stats = self.simulate_constant_angular_velocity(needed_av, end_time, True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < aa_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p,i,d)
        if best_stats and best_stats.speed < aa_threshold:
            self.avPID.setPID(*best_pid)
            # print 'avPID: %s\n%s\n' % (best_pid, best_stats)
            # self.analyze_results(self.simulate_constant_angular_velocity(needed_av, end_time*2))
            return best_pid
        return None

    def optimize_at_PID(self, start_error, av_threshold, step, end_time, P, I, D):
        best_pid = None
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                for d in np.linspace(D[0], D[1], step):
                    self.atPID.setPID(p,i,d)
                    time, error, thrust, zero_stats = self.simulate_static_attitude(start_error, end_time, True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < av_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p,i,d)
        if best_stats and best_stats.speed < av_threshold:
            self.atPID.setPID(*best_pid)
            # print 'atPID: %s\n%s\n' % (best_pid, best_stats)
            # self.analyze_results(self.simulate_static_attitude(start_error, end_time*2))
            return best_pid
        return None

    @classmethod
    def analyze_results(self, results):
        time, error, thrust, zero_stats = results
        if zero_stats:
            print '\n%s\n' % str(zero_stats)
        print '=' * 80
        plt.subplot(2, 1, 1)
        plt.plot(time, error)
        plt.subplot(2, 1, 2)
        plt.plot(time, thrust)
        plt.show()


class Main(MPMain):
    class Craft(object):
        lever = 4
        maxThrust = 120
        def __init__(self, maxAA, accelSpeed, decelSpeed):
            at_pid = PID2(0, 0, 0, -1, 1)
            av_pid = PID2(0, 0, 0, -3, 3)
            engine = Engine(self.maxThrust, accelSpeed, decelSpeed)
            self.atc = ATC(engine, self.lever, self.maxThrust*self.lever/maxAA, at_pid, av_pid, 0.8)

        def optimize(self):
            if not self.atc.optimize_av_PID(0.5, 0.06, 10, 10, (0.1, 2), (0, 0.2), (0, 2)): return False
            if not self.atc.optimize_at_PID(5, ATC.tenth_deg, 10, 10, (0.1, 2), (0, 0.2), (0, 2)): return False
            return True

        def pack(self):
            return ([self.atc.MaxAA, self.atc.engineF.acceleration, self.atc.engineF.deceleration]+
                    self.atc.avPID.pack() + self.atc.atPID.pack())


    def _main(self):
        maxAA = np.linspace(0.1, 10, 10)
        accelSpeed = np.linspace(0, 1, 10)
        decelSpeed = np.linspace(0, 1, 10)
        def mapper(index, params):
            print '%d ' % index
            craft = self.Craft(*params[index])
            if craft.optimize():
                pack = craft.pack()
                return pack
            return None
        work = np.array(np.meshgrid(maxAA, accelSpeed, decelSpeed)).T.reshape(-1, 3)
        print 'Checking %d craft configurations...' % work.shape[0]
        results = parallelize_work(self.abort_event, True, 1, mapper, range(0, work.shape[0]), work)
        if results:
            with open('ATC-stats.csv', 'wb') as out:
                writer = csv.writer(out)
                writer.writerow(('maxAA', 'accelSpeed', 'decelSpeed', 'AV P', 'AV I', 'AV D', 'AT P', 'AT I', 'AT D'))
                writer.writerows((r for r in results if r is not None))
        return 0

stage = 1
if __name__ == '__main__':
    if stage == 0:
        ###############################################################################################
        gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
        game = u'KSP_test_1.2.1'
        gamedata = u'GameData'
        def gamefile(filename): return os.path.join(gamedir, game, filename)
        def datafile(filename): return os.path.join(gamedir, game, gamedata, filename)

        wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
        wiplash = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineTurbo.cfg'))

        at_pid = PID2(2, 0.0, 0.1, -1, 1)
        av_pid = PID2(1, 0.0, 1.1, -3, 3)

        def square_lever(lever):
            return np.sqrt(lever[0]**2+lever[1]**2)

        # Jet Hangar Test.AttitudeControl:
        # MoI: (747.4614, 83.5321, 789.1782); | v | = 1090.174
        # lever: (-1.83199, 3.203556, 3.024457); | v | = 4.771405
        # maxThrust: 130,
        # lever: (1.836533, 3.203599, 3.021605); | v | = 4.771372
        # maxThrust: 130,
        # lever: (-1.830195, -3.227458, 3.020665); | v | = 4.784403
        # maxThrust: 130,
        # lever: (1.836236, -3.225128, 3.017829); | v | = 4.783357
        # maxThrust: 130
        heavy = ATC(wiplash, 3.203599, 747.4614, at_pid, av_pid, 0.8)

        # JetTest.AttitudeControl:
        # MoI: (71.95605, 67.7888, 127.0972); | v | = 161.0177
        # Engines: [
        #     lever: (1.87511, -1.744024, 2.343808); | v | = 3.471468
        # maxThrust: 120,
        #            lever: (-1.875131, -1.744012, 2.343805); | v | = 3.471471
        # maxThrust: 120,
        #            lever: (1.877351, 1.721185, 2.343811); | v | = 3.461268
        # maxThrust: 120,
        #            lever: (-1.877346, 1.721192, 2.343807); | v | = 3.461266
        # maxThrust: 120
        light = ATC(wheesly, 1.877351, 71.99494, at_pid, av_pid, 0.8)

        light.optimize_av_PID(0.5, 0.06, 10, 5, (0.1, 2), (0, 0.2), (0, 1))
        light.optimize_at_PID(5, ATC.tenth_deg, 10, 10, (0.1, 2), (0, 0.2), (0, 2))


        ATC.analyze_results(light.simulate_constant_angular_velocity(1, 60))
        ATC.analyze_results(light.simulate_static_attitude(15, 60))

        # heavy.optimize_av_PID(0.5, 0.04, 10, 10, (0.1, 2), (0, 0.2), (0, 1))
        # heavy.optimize_at_PID(5, ATC.tenth_deg, 10, 10, (0.1, 2), (0, 0.2), (0, 2))
        #
        # ATC.analyze_results(heavy.simulate_constant_angular_velocity(1, 60))
        # ATC.analyze_results(heavy.simulate_static_attitude(15, 60))
    elif stage == 1:
        Main(run=True)
    else:
        from sklearn.decomposition import PCA
        import pandas as pd
        df = pd.read_csv('ATC-stats-10.csv')
        params = df.ix[:,0:3]
        av_pids = df.ix[:,3:6]
        at_pids = df.ix[:, 6:]

        def do_pca(data, y):
            pca = PCA(whiten=False)
            pca.fit(data)
            print pca.components_[:2]
            print pca.explained_variance_ratio_[:2]
            data_r = pca.transform(data)
            cnorm = np.max(y)
            def color(x):
                xn = x/cnorm
                return (1-xn, xn, 0)
            plt.scatter(data_r[:, 0], data_r[:, 1], c=[color(_y) for _y in y])
            plt.show()

        do_pca(av_pids, df['maxAA'])
        do_pca(at_pids, df['maxAA'])
