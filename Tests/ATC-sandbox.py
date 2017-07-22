import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA
from scipy.optimize import curve_fit
import pandas as pd

from common import clampL, clampH, clamp01, clamp, lerp, PID, PID2, plt_show_maxed

from itertools import combinations_with_replacement
from KSPUtils import Part, SearchTerm
from BioUtils.Tools.Multiprocessing import MPMain, parallelize_work

dt = 0.02


class Engine(object):
    def __init__(self, maxThrust, acceleration=0, deceleration=0):
        self.name = 'Engine'
        self.maxThrust = maxThrust
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.instant = acceleration == 0 and deceleration == 0
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

    def __init__(self, engine, lever, MoI, atPID, avPID, base_level, on_update=None):
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
        self.instant = self.engineF.instant
        self.MoI = MoI
        self.MaxAA = self.engineF.maxThrust*self.engineF.lever/self.MoI
        self.on_update = on_update
        self.AV = 0
        self.AA = 0

    def reset(self):
        self.AV = 0
        self.AA = 0
        self.engineF.limit = self.base_level
        self.engineF.thrust = self.engineF.maxThrust * self.base_level
        self.engineR.limit = self.base_level
        self.engineR.thrust = self.engineF.maxThrust * self.base_level

    def updateAV(self):
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
        if self.on_update is not None:
            self.on_update(self)
        else:
            self.atPID.update(self.error)
            self.avPID.update(self.atPID.action - self.AV)
        self.updateAV()
        self.error -= self.AV*dt
        self.error %= self.twoPi
        if self.error > np.pi: self.error -= self.twoPi
        elif self.error < -np.pi: self.error += self.twoPi

    class ZeroStats(object):
        def __init__(self, time, speed, desc, units):
            self.time = time
            self.speed = speed
            self.desc = desc or 'Speed'
            self.units = units
            self.metric = self.time+self.speed*100

        def __str__(self):
            return ('Zero at: %f s\n'
                    '%s: %f %s' % (self.time, self.desc, self.speed, self.units))

    def simulate_constant_angular_velocity(self, needed_av, end_time, end_on_zero=False):
        t = 0
        time = [0]
        error = [needed_av]
        limit = [self.engineF.limit]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while t < end_time:
            t += dt
            av_error = needed_av-self.AV
            if self.on_update is not None:
                self.on_update(self)
            else:
                self.avPID.update(av_error)
            self.updateAV()
            if not zero_stats and (av_error < self.tenth_deg or av_error*error[-1] < 0):
                zero_stats = self.ZeroStats(t, abs(self.AA), 'AA', 'rad/s2')
            error.append(av_error)
            limit.append(self.engineF.limit)
            thrust.append(self.engineF.thrust / self.engineF.maxThrust)
            time.append(t)
            if zero_stats and end_on_zero: break
        return 'dAV [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, limit, thrust, zero_stats

    tenth_deg = 0.1/180*np.pi

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        t = 0
        time = [0]
        error = [self.error]
        limit = [self.engineF.limit]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while t < end_time:
            t += dt
            self.update()
            # print 'Time: %f' % t
            # print 'AT PID: ' + str(self.atPID)
            # print 'AV PID: ' + str(self.avPID) + '\n'
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(t, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            limit.append(self.engineF.limit)
            thrust.append(self.engineF.thrust / self.engineF.maxThrust)
            time.append(t)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, limit, thrust, zero_stats

    def optimize_av_PID(self, needed_av, aa_threshold, step, end_time, P, I, D):
        best_pid = None
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                if self.instant: D = (0,0)
                for d in np.linspace(D[0], D[1], step):
                    self.avPID.setPID(p,i,d)
                    name, time, error, limit, thrust, zero_stats = self.simulate_constant_angular_velocity(needed_av,
                                                                                                     end_time, True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < aa_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p,i,d)
        if best_stats and best_stats.speed < aa_threshold:
            self.avPID.setPID(*best_pid)
            print 'avPID: %s\n%s\n' % (best_pid, best_stats)
            # self.analyze_results(self.simulate_constant_angular_velocity(needed_av, end_time*2))
            return best_pid
        return None

    def optimize_av_D(self, needed_av, aa_threshold, step, end_time, D):
        best_pid = None
        best_stats = None
        for d in np.linspace(D[0], D[1], step):
            self.avPID.D = d
            name, time, error, limit, thrust, zero_stats = self.simulate_constant_angular_velocity(needed_av, end_time,
                                                                                               True)
            if not zero_stats: continue
            if not best_stats or zero_stats.speed < aa_threshold and zero_stats.metric < best_stats.metric:
                best_stats = zero_stats
                best_pid = (self.avPID.P, self.avPID.I, d)
        if best_stats and best_stats.speed < aa_threshold:
            self.avPID.setPID(*best_pid)
            print 'avPID: %s\n%s\n' % (best_pid, best_stats)
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
                    name, time, error, limit, thrust, zero_stats = self.simulate_static_attitude(start_error, end_time,
                                                                                             True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < av_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p,i,d)
        if best_stats and best_stats.speed < av_threshold:
            self.atPID.setPID(*best_pid)
            print 'atPID: %s\n%s\n' % (best_pid, best_stats)
            # self.analyze_results(self.simulate_static_attitude(start_error, end_time*2))
            return best_pid
        return None

    @classmethod
    def analyze_results(cls, cols, col, *results):
        for result in results:
            name, time, error, limit, thrust, zero_stats = result
            if zero_stats:
                print '\n%s\n' % str(zero_stats)
            print '=' * 80
            plt.subplot(3, cols, col)
            plt.plot(time, error, label=name)
            plt.legend()
            plt.subplot(3, cols, cols+col)
            plt.plot(time, limit)
            plt.subplot(3, cols, cols*2+col)
            plt.plot(time, thrust)
            plt.xlabel('time')
            plt.ylabel('thrust')
        # plt.show()


class Main(MPMain):
    class Craft(object):
        lever = 4
        maxThrust = 120

        def __init__(self, maxAA, accelSpeed):
            at_pid = PID2(0, 0, 0, -np.pi, np.pi)
            av_pid = PID(0, 0, 0, -1, 1)
            engine = Engine(self.maxThrust, accelSpeed, accelSpeed*2)
            self.atc = ATC(engine, self.lever, self.maxThrust*self.lever/maxAA*2, at_pid, av_pid, 0)

        def optimize(self):
            if not self.atc.optimize_av_PID(0.5, 0.06, 20, 10, (0.1, 10), (0, 0.2), (0, 10)): return False
            if not self.atc.optimize_at_PID(5, ATC.tenth_deg, 20, 10, (0.1, 10), (0, 0.2), (0, 10)): return False
            return True

        def pack(self):
            return ([self.atc.MaxAA, self.atc.engineF.acceleration]+
                    self.atc.avPID.pack() + self.atc.atPID.pack())

    def _main(self):
        maxAA = np.linspace(0.001, 2, 100)
        accelSpeed = np.linspace(0, 0, 1)

        def mapper(index, params):
            print '%d ' % index
            craft = self.Craft(*params[index])
            if craft.optimize():
                pack = craft.pack()
                return pack
            return None
        work = np.array(np.meshgrid(maxAA, accelSpeed)).T.reshape(-1, 2)
        print 'Checking %d craft configurations...' % work.shape[0]
        results = parallelize_work(self.abort_event, True, 1, mapper, range(0, work.shape[0]), work)
        if results:
            with open('ATC-stats.csv', 'wb') as out:
                writer = csv.writer(out)
                writer.writerow(('maxAA', 'accelSpeed', 'AV_P', 'AV_I', 'AV_D', 'AT_P', 'AT_I', 'AT_D'))
                writer.writerows((r for r in results if r is not None))
        return 0

gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
game = u'KSP_test_1.3'
gamedata = u'GameData'

def gamefile(filename): return os.path.join(gamedir, game, filename)
def datafile(filename): return os.path.join(gamedir, game, gamedata, filename)

stage = 3
if __name__ == '__main__':
    if stage == 0:
        wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
        wiplash = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineTurbo.cfg'))
        rocket = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T45/liquidEngineLV-T45.cfg'))

        at_pid = PID2(2, 0.0, 0.1, -np.pi, np.pi)
        av_pid = PID(1, 0.0, 1.1, -1, 1)

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

        rocket = ATC(rocket, 3.203599, 747.4614, at_pid, av_pid, 0.5)

        rocket.optimize_av_PID(0.5, 0.06, 20, 10, (0.1, 10), (0, 0), (0, 10))
        rocket.optimize_at_PID(5, ATC.tenth_deg, 20, 10, (0.1, 10), (0, 0.2), (0, 10))

        ATC.analyze_results(rocket.simulate_constant_angular_velocity(1, 60))

        ATC.analyze_results(rocket.simulate_static_attitude(150, 60))

        # heavy.optimize_av_PID(0.5, 0.04, 10, 10, (0.1, 2), (0, 0.2), (0, 1))
        # heavy.optimize_at_PID(5, ATC.tenth_deg, 10, 10, (0.1, 2), (0, 0.2), (0, 2))
        #
        # ATC.analyze_results(heavy.simulate_constant_angular_velocity(1, 60))
        # ATC.analyze_results(heavy.simulate_static_attitude(15, 60))
    elif stage == 1:
        Main(run=True)
    elif stage == 2:
        df = pd.read_csv('ATC-stats.csv')
        params = df.ix[:,0:2]
        av_pids = df.ix[:,2:5]
        at_pids = df.ix[:, 5:]

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

        rocket = df[df.accelSpeed == 0]
        jets = df[df.accelSpeed > 0]

        def model(x, a,b): return a/(b+x)
        abc, conv = curve_fit(model, rocket.maxAA, rocket.AV_P)
        print 'AV.P(aa): %f/(%f+aa)' % tuple(abc)

        plt.plot(rocket.maxAA, rocket.AV_P, '.-')
        plt.plot(rocket.maxAA, rocket.AV_D, 'r.-')
        plt.plot(rocket.maxAA, model(rocket.maxAA, *abc))
        plt.show()

        plt.plot(rocket.maxAA, rocket.AT_P, '.-')
        plt.plot(rocket.maxAA, rocket.AT_D, 'r.-')
        # plt.plot(rocket.maxAA, model(rocket.maxAA, *abc))
        plt.show()

        # def model(x, a, b, c): return a ** (b * x + c)
        # abc, conv = curve_fit(model, rocket_prim.maxAA, rocket_prim['AV_P'])
        # plt.plot(np.linspace(2, 9.5, 20), model(np.linspace(0.1, 10, 20), *abc))


        # plt.plot(df[df.accelSpeed > 0].maxAA, df[df.accelSpeed > 0]['AV P'])
        # plt.plot(df[df.accelSpeed > 0].maxAA, df[df.accelSpeed > 0]['AV D'])



        # do_pca(av_pids, (df['accelSpeed'] == 0))
        # do_pca(at_pids, df['maxAA'])
        # do_pca(df.ix[:,2:][df.accelSpeed == 0], df['maxAA'])
    elif stage == 3:
        class MinMaxK(object):
            def __init__(self, Min, Max, K):
                self.Min = float(Min)
                self.Max = float(Max)
                self.K = float(K)
                
            def apply(self, val):
                return clamp(val*self.K, self.Min, self.Max)
        
        atP_clamp = MinMaxK(1, 5e10, 10)
        avP_clamp = MinMaxK(1, 5, 0.5)

        atD_Curve = 1
        atD_Scale = 3
        atD_Clamp = 0

        atI_Scale = 1
        atI_AV_Scale = 10
        atI_ErrThreshold = 0.8
        atI_ErrCurve = 2
        atP_ErrCurve = 0.2

        atErrThreshold = 0.8
        atErrScale = 100

        avI_Scale = 0.4

        AxisCorrection = 0.2

        SlowTorqueF = 0.06
        MaxSlowF = 3

        atClamp = np.pi * 10
        avClamp = 1000

        lever = 4
        wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
        LV_T30 = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T30/liquidEngineLV-T30.cfg'))
        stats = []


        def simAA(cols, col, error, maxAA, engine):
            def on_update(atc):
                Err = atc.error/np.pi
                iErr = 1-Err
                atI_iErr = clampL(iErr - atI_ErrThreshold, 0)
                imaxAA = 1/maxAA
                L = abs(atc.AV*atc.MoI)
                atP = atP_clamp.apply(imaxAA * (1 + clampL(iErr - atErrThreshold, 0) * atErrScale))
                atD = clampL(atP - ((maxAA - 1)**atD_Curve) * atD_Scale, atD_Clamp)
                atD *= clampH(iErr + (1 - clampH(maxAA, 1)) + L, 1)
                avP = clampL(avP_clamp.K * imaxAA, avP_clamp.Min)
                avD = 0
                slow = 1
                if not engine.instant:
                    slow = clampH(1 + max(engine.acceleration, engine.deceleration)*atc.engineF.lever*SlowTorqueF,
                                  MaxSlowF)
                    avD = slow*slow
                    avP = 1
                else:
                    atD /= atP
                    atP = clampH(200 * maxAA * clampL(iErr, 0.1), 1+atI_iErr**atP_ErrCurve)
                if iErr < atI_ErrThreshold or atc.AV*atc.error < 0:
                    atc.atPID.I = 0
                    atc.atPID.ierror = 0
                else:
                    atI_iErr = atI_iErr**atI_ErrCurve
                    atc.atPID.I = atI_Scale * maxAA * atI_iErr / (1+abs(atc.AV)*atI_AV_Scale*atI_iErr)
                atc.atPID.P = atP
                atc.atPID.D = atD
                atc.avPID.P = avP
                atc.avPID.I = avI_Scale * avP
                atc.avPID.D = avD
                atc.atPID.update2(atc.error, -atc.AV)
                avErr = atc.AV - atc.atPID.action + atc.AV*avP*AxisCorrection
                atc.avPID.update(-avErr)
                atc.avPID.action /= slow
                # print "\nAA: %.2f" % maxAA
                # print "AV %f, error %f" % (atc.AV, atc.error/np.pi)
                # print "AT: "+str(at_pid)
                # print "AV: "+str(av_pid)

            at_pid = PID2(1, 0.0, 1, -atClamp, atClamp)
            av_pid = PID(1, 0.0, 1, -avClamp, avClamp)
            atc = ATC(engine, lever, engine.maxThrust*lever/maxAA*2, at_pid, av_pid, 0.0, on_update)
            # atc.optimize_av_D(1, ATC.tenth_deg, 30, 60, (0.1*avAAf, 10*avAAf))
            # st.append(atc.avPID.P)
            # st.append(atc.avPID.D)

            ATC.analyze_results(cols, col,
                                (atc.simulate_static_attitude(error, clampL(error*2, 60))))

        def simAngle(eng, *angles, **kwargs):
            cols = len(angles)
            AA = kwargs.get('AA', (0.01, 10, 10))
            for c, ang in enumerate(angles):
                [simAA(cols, c+1, ang, aa, eng) for aa in np.linspace(*AA)]
            plt_show_maxed()

        def simSpeed(eng, ang, *mults):
            cols = len(mults)
            for c, m in enumerate(mults):
                e = eng.clone()
                e.acceleration *= m
                e.deceleration *= m
                [simAA(cols, c+1, ang, aa, e) for aa in np.linspace(0.01, 2, 10)]
            plt_show_maxed()


        # simSpeed(wheesly, 90, 0.2, 1, 3)
        # simAngle(wheesly, 45, 30, 15, 3, AA=(0.1, 1, 5))
        simAngle(LV_T30, 175, 60, 15, 3)

        # force = False
        # fname = 'avD.csv'
        # if force or not os.path.isfile(fname):
        #     simSpeed(wheesly, 135, 0.2, 0.5, 1, 1.5, 2)
        #     df = pd.DataFrame.from_records(stats, columns=('maxAA', 'avAAf', 'ADspeed', 'avD'))
        #     df.to_csv(fname)
        # else:
        #     df = pd.read_csv(fname)
        # print df.to_string()
        # bounds = 8, 10
        # df1 = df[df.avAAf > bounds[0]][df.avAAf < bounds[1]]
        # def model((AAf, AD), a, b):
        #     # return AAf*a*AD**b
        #     return AAf * 0.74 * AD ** -0.265 +(a*AD)**(b*(AAf-bounds[0]+0.1))
        #     # return (a*AD)**((AAf-bound+0.1)*b)+(c*AD)**((AAf-bound+0.1)*d)# + c/(AAf-bound+1)
        # abc, conv = curve_fit(model, (df1.avAAf, df1.ADspeed), df1.avD)
        # print abc
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(df.avAAf, df.ADspeed, df.avD, c='red')
        # AAf, AD = np.mgrid[bounds[0]:bounds[1]:0.1, 0.02:0.25:0.02]
        # ax.plot_wireframe(AAf, AD, model((AAf, AD), *abc))
        # plt_show_maxed()

        # df.sort_values('avAAf')
        # plt.plot(df.avAAf, df.avD, '.')
        # plt_show_maxed()
    else: print "No %d stage" % stage
