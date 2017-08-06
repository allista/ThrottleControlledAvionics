import os
import csv
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

import sys
from sklearn.decomposition import PCA
from scipy.optimize import curve_fit
import pandas as pd

from common import clampL, clampH, clamp, lerp, PID, PID2, plt_show_maxed, color_grad

from KSPUtils import Part, SearchTerm
# from BioUtils.Tools.Multiprocessing import MPMain, parallelize_work

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
        elif delta > 0:
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

    def __init__(self, engine, lever, MoI, atPID, avPID, base_level, on_update=None, wheels_torque=0):
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
        self.wheels = wheels_torque
        self.MoI = MoI
        self.EnginesMaxAA = self.engineF.maxThrust*base_level*self.engineF.lever/self.MoI
        self.WheelsMaxAA = self.wheels/self.MoI
        self.MaxAA = self.EnginesMaxAA+self.WheelsMaxAA
        self.WheelsRatio = self.WheelsMaxAA/self.MaxAA
        self.InstantRatio = 1.0 if self.instant else self.WheelsMaxAA/self.MaxAA
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
            # self.engineF.limit = min(self.base_level + self.avPID.action, 1)
            self.engineF.limit = self.base_level
            self.engineR.limit = max(self.base_level - self.avPID.action, 0)
        else:
            # self.engineR.limit = min(self.base_level - self.avPID.action, 1)
            self.engineR.limit = self.base_level
            self.engineF.limit = max(self.base_level + self.avPID.action, 0)
        self.engineF.update()
        self.engineR.update()
        self.AA = (self.engineF.torque - self.engineR.torque + self.wheels*self.avPID.action) / self.MoI
        self.AV += self.AA * dt

    def update(self):
        if self.on_update is not None:
            self.on_update(self)
        else:
            self.atPID.update(abs(self.error))
            self.avPID.update(self.atPID.action * np.sign(self.error) - self.AV)
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
        action = [self.avPID.action]
        thrust = [(self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust]
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
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(t)
            if zero_stats and end_on_zero: break
        return 'dAV [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    tenth_deg = 0.1/180*np.pi

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        t = 0
        time = [0]
        error = [self.error]
        action = [self.avPID.action]
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
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(t)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

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
        colors = color_grad(len(results))
        for i, result in enumerate(results):
            name, time, error, action, thrust, zero_stats = result
            if zero_stats:
                print '\n%s\n' % str(zero_stats)
            print '=' * 80
            plt.subplot(3, cols, col)
            plt.plot(time, error, label=name, color=colors[i])
            plt.ylabel('error (deg)')
            plt.legend()
            plt.subplot(3, cols, cols+col)
            plt.plot(time, action, color=colors[i])
            plt.ylabel('action')
            plt.subplot(3, cols, cols*2+col)
            plt.plot(time, thrust, color=colors[i])
            plt.xlabel('time')
            plt.ylabel('thrust')
        # plt.show()

#
# class Main(MPMain):
#     class Craft(object):
#         lever = 4
#         maxThrust = 120
#
#         def __init__(self, maxAA, accelSpeed):
#             at_pid = PID2(0, 0, 0, -np.pi, np.pi)
#             av_pid = PID(0, 0, 0, -1, 1)
#             engine = Engine(self.maxThrust, accelSpeed, accelSpeed*2)
#             self.atc = ATC(engine, self.lever, self.maxThrust*self.lever/maxAA*2, at_pid, av_pid, 0)
#
#         def optimize(self):
#             if not self.atc.optimize_av_PID(0.5, 0.06, 20, 10, (0.1, 10), (0, 0.2), (0, 10)): return False
#             if not self.atc.optimize_at_PID(5, ATC.tenth_deg, 20, 10, (0.1, 10), (0, 0.2), (0, 10)): return False
#             return True
#
#         def pack(self):
#             return ([self.atc.MaxAA, self.atc.engineF.acceleration]+
#                     self.atc.avPID.pack() + self.atc.atPID.pack())
#
#     def _main(self):
#         maxAA = np.linspace(0.001, 2, 100)
#         accelSpeed = np.linspace(0, 0, 1)
#
#         def mapper(index, params):
#             print '%d ' % index
#             craft = self.Craft(*params[index])
#             if craft.optimize():
#                 pack = craft.pack()
#                 return pack
#             return None
#         work = np.array(np.meshgrid(maxAA, accelSpeed)).T.reshape(-1, 2)
#         print 'Checking %d craft configurations...' % work.shape[0]
#         results = parallelize_work(self.abort_event, True, 1, mapper, range(0, work.shape[0]), work)
#         if results:
#             with open('ATC-stats.csv', 'wb') as out:
#                 writer = csv.writer(out)
#                 writer.writerow(('maxAA', 'accelSpeed', 'AV_P', 'AV_I', 'AV_D', 'AT_P', 'AT_I', 'AT_D'))
#                 writer.writerows((r for r in results if r is not None))
#         return 0

gamedir = u'/media/user/Lir\'s/allis/AT_KSP_Plugins/KSP-test/'
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
    # elif stage == 1:
    #     Main(run=True)
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

        # def model(x, a,b): return a/(b+x)
        # abc, conv = curve_fit(model, rocket.maxAA, rocket.AV_P)
        # print 'AV.P(aa): %f/(%f+aa)' % tuple(abc)
        #
        # plt.plot(rocket.maxAA, rocket.AV_P, '.-')
        # plt.plot(rocket.maxAA, rocket.AV_D, 'r.-')
        # plt.plot(rocket.maxAA, IR_vs_avD(rocket.maxAA, *abc))
        # plt.show()
        #
        # plt.plot(rocket.maxAA, rocket.AT_P, '.-')
        # plt.plot(rocket.maxAA, rocket.AT_D, 'r.-')
        # # plt.plot(rocket.maxAA, model(rocket.maxAA, *abc))
        # plt.show()

        # def model(x, a, b, c): return a ** (b * x + c)
        # abc, conv = curve_fit(model, rocket_prim.maxAA, rocket_prim['AV_P'])
        # plt.plot(np.linspace(2, 9.5, 20), model(np.linspace(0.1, 10, 20), *abc))


        # plt.plot(df[df.accelSpeed > 0].maxAA, df[df.accelSpeed > 0]['AV P'])
        # plt.plot(df[df.accelSpeed > 0].maxAA, df[df.accelSpeed > 0]['AV D'])



        # do_pca(av_pids, (df['accelSpeed'] == 0))
        # do_pca(at_pids, df['maxAA'])
        # do_pca(df.ix[:,2:][df.accelSpeed == 0], df['maxAA'])
    elif stage == 3:
        class FastConfig(object):
            atP_ErrThreshold = 0.7
            atP_ErrCurve = 0.5

            atP_LowAA_Scale = 1.2
            atP_LowAA_Curve = 0.8
            atD_LowAA_Scale = 1
            atD_LowAA_Curve = 0.5

            atP_HighAA_Scale = 0.2
            atP_HighAA_Curve = 0.3
            atP_HighAA_Max = 4
            atD_HighAA_Scale = 1.0
            atD_HighAA_Curve = 0.4

            atI_Scale = 1
            atI_AV_Scale = 10
            atI_ErrThreshold = 0.8
            atI_ErrCurve = 2

            avP_MaxAA_Intersect = 5
            avP_MaxAA_Inclination = 0.4
            avP_MaxAA_Curve = 0.8
            avP_Min = 0.2

            avI_Scale = 0.4

        def tune_steering_fast(cfg, atc, iErrf, imaxAA, AM):
            """
            :param cfg: Configuration object
            :type cfg: FastConfig
            :param atc: Attitude Controll model
            :type atc: ATC
            :param iErrf: inverse error factor
            :type iErrf: float
            :param imaxAA: inverse maxAA
            :type imaxAA: float
            :param AM: angular momentum
            :type AM: float
            """
            # compute coefficients of attitude PID
            atP_iErrf = clampL(iErrf - cfg.atP_ErrThreshold, 0) ** cfg.atP_ErrCurve
            if atc.MaxAA >= 1:
                atP = clampH(1
                             + cfg.atP_HighAA_Scale * atc.MaxAA ** cfg.atP_HighAA_Curve
                             + atP_iErrf, cfg.atP_HighAA_Max)
                atD = cfg.atD_HighAA_Scale * imaxAA ** cfg.atD_HighAA_Curve * clampH(iErrf + abs(AM), 1.2)
            else:
                atP = (1
                       + cfg.atP_LowAA_Scale * atc.MaxAA ** cfg.atP_LowAA_Curve
                       + atP_iErrf)
                atD = cfg.atD_LowAA_Scale * imaxAA ** cfg.atD_LowAA_Curve * clampH(iErrf + abs(AM), 1.2)
            # compute coefficients of angular velocity PID
            avP = clampL(cfg.avP_MaxAA_Intersect -
                         cfg.avP_MaxAA_Inclination * atc.MaxAA ** cfg.avP_MaxAA_Curve,
                         cfg.avP_Min)
            # tune PIDS and compute steering
            tune_pids_set_steering(cfg, atc, iErrf, atP, atD, avP, 0)

        # Valid for InstantRatios >= 0.3
        class MixedConfig3plus(object):
            atP_ErrThreshold = 0.8
            atP_ErrCurve = 0.5

            atP_LowAA_Scale = 0.2
            atP_LowAA_Curve = 1.2
            atD_LowAA_Scale = 1.2
            atD_LowAA_Curve = 0.6

            atP_HighAA_Scale = 0.2
            atP_HighAA_Curve = 0.3
            atP_HighAA_Max = 4
            atD_HighAA_Scale = 1.0
            atD_HighAA_Curve = 0.4

            atI_Scale = 1
            atI_AV_Scale = 10
            atI_ErrThreshold = 0.8
            atI_ErrCurve = 2

            avP_MaxAA_Intersect = 5
            avP_MaxAA_Inclination = 0.4
            avP_MaxAA_Curve = 0.8
            avP_Min = 0.2

            avI_Scale = 0.4

        class MixedConfig(object):
            atI_Scale = 0
            atI_AV_Scale = 10
            atI_ErrThreshold = 0.8
            atI_ErrCurve = 2

            avP_A = 100
            avP_B = 0.04
            avP_C = -100
            avP_D = 0.7

            avD_A = 0.65
            avD_B = -0.01
            avD_C = 0.5
            avD_D = 0.7

            avI_Scale = 0.0

        def tune_steering_mixed(atc, iErrf, imaxAA, AM):
            if atc.InstantRatio > 0.3:
                tune_steering_fast(MixedConfig3plus, atc, iErrf, imaxAA, AM)
            else:
                atP = 1#*clampL(atc.EnginesMaxAA**0.5, 1)
                atD = 0
                avP = ((MixedConfig.avP_A / (atc.InstantRatio ** MixedConfig.avP_D + MixedConfig.avP_B) +
                        MixedConfig.avP_C) / (1 + abs(AM)) / atc.MaxAA)
                avD = ((MixedConfig.avD_A / (atc.InstantRatio ** MixedConfig.avD_D + MixedConfig.avD_B) +
                        MixedConfig.avD_C) / atc.MaxAA)
                # tune PIDS and compute steering
                tune_pids_set_steering(MixedConfig, atc, iErrf, atP, atD, avP, avD)

        #### Analysis of avP/avD coefficients for mixed torque system ####
        #
        # avP_curve = [(0.01, 4.2, 500),
        #              (0.02, 2.8, 360),
        #              (0.05, 2.1, 240), #280
        #              (0.1,  1.6, 170),
        #              (0.2,  1.4, 120),
        #              ]
        # avP_curve = np.array(avP_curve, float)
        #
        # def IR_vs_avD(x, a, b, c): return a / (b + x**0.1) + c
        # opt, cov = curve_fit(IR_vs_avD, avP_curve[:, 0], avP_curve[:, 1])
        # print opt
        #
        # avD_mod = IR_vs_avD(avP_curve[:, 0], *opt)
        # plt.plot(avP_curve[:, 0], avP_curve[:, 1], '.-')
        # plt.plot(avP_curve[:, 0], avD_mod, '--')
        # plt.show()
        #
        # def IR_vs_avP(x, a, b, c): return a/(x**0.1+b)+c
        # opt, cov = curve_fit(IR_vs_avP, avP_curve[:,0], avP_curve[:, 2])
        # print opt
        #
        # plt.plot(avP_curve[:,0], avP_curve[:,2], '.-')
        # plt.plot(avP_curve[:,0], IR_vs_avP(avP_curve[:,0], *opt), '--')
        # plt.show()
        ###################################################################

        class SlowConfig(object):
            atI_Scale = 0.0
            atI_AV_Scale = 10
            atI_ErrThreshold = 0.8
            atI_ErrCurve = 2

            avP_HighAA_Scale = 5
            avD_HighAA_Intersect = 10
            avD_HighAA_Inclination = 2
            avD_HighAA_Max = 2

            avP_LowAA_Scale = 8
            avD_LowAA_Intersect = 25
            avD_LowAA_Inclination = 10

            avI_Scale = 0.0

            SlowTorqueF = 0.2

        def tune_steering_slow(atc, iErrf, imaxAA, AM):
            atP = 1
            atD = 0
            slowF = (1 + SlowConfig.SlowTorqueF / max(atc.engineF.acceleration, atc.engineF.deceleration))
            if atc.MaxAA >= 1:
                avP = SlowConfig.avP_HighAA_Scale/slowF
                avD = clampL(SlowConfig.avD_HighAA_Intersect - SlowConfig.avD_HighAA_Inclination * atc.MaxAA, SlowConfig.avD_HighAA_Max)
            else:
                avP = SlowConfig.avP_LowAA_Scale/slowF
                avD = SlowConfig.avD_LowAA_Intersect - SlowConfig.avD_LowAA_Inclination * atc.MaxAA
            # tune PIDS and compute steering
            tune_pids_set_steering(SlowConfig, atc, iErrf, atP, atD, avP, avD)

        wheels_ratio = 0.4
        AAs = 0.3, 0.7, 0.9, 1.5, 3, 9
        angles = 85, 25, 3

        def tune_pids_set_steering(cfg, atc, iErrf, atP, atD, avP, avD):
            atI_iErrf = clampL(iErrf - cfg.atI_ErrThreshold, 0)
            overshot = atc.AV*atc.error < 0
            if atI_iErrf <= 0 or overshot:
                atc.atPID.I = 0
                atc.atPID.ierror = 0
            else:
                atI_iErrf = atI_iErrf**cfg.atI_ErrCurve
                atc.atPID.I = cfg.atI_Scale * atc.MaxAA * atI_iErrf \
                    /(1 + clampL(atc.AV*np.sign(atc.error), 0) * cfg.atI_AV_Scale * atI_iErrf)
            atc.atPID.P = atP
            atc.atPID.D = atD
            atc.avPID.P = avP
            atc.avPID.D = avD
            atc.avPID.I = cfg.avI_Scale * avP
            atc.atPID.update2(abs(atc.error), -atc.AV)
            avErr = abs(atc.atPID.action) * np.sign(atc.error) - atc.AV
            atc.avPID.update(avErr)
            atc.avPID.action = clamp(atc.avPID.action, -1, 1)

        def tune_steering2(atc):
            """
            :param atc: attitude controller
            :type atc: ATC
            """
            iErrf = 1 - abs(atc.error/np.pi)
            imaxAA = 1 / atc.MaxAA
            AM = atc.AV*atc.MoI
            if atc.InstantRatio > 0.7:
                tune_steering_fast(FastConfig, atc, iErrf, imaxAA, AM)
            elif atc.InstantRatio >= 0.005:
                tune_steering_mixed(atc, iErrf, imaxAA, AM)
            else:
                tune_steering_slow(atc, iErrf, imaxAA, AM)
            print ('atc.MaxAA %f, err %f, av %f, iErr %f\natPID %s\navPID %s' %
                   (atc.MaxAA, atc.error / np.pi * 180, atc.AV / np.pi * 180, iErrf, atc.atPID, atc.avPID))

        lever = 4

        def simAA(error, maxAA, engine, base_thrust, wheels_ratio):
            at_pid = PID2(1, 0.0, 1, 0, np.pi*10)
            av_pid = PID(1, 0.0, 1, -1000, 1000)
            if wheels_ratio < 1:
                MoI = engine.maxThrust*lever*2/maxAA/(1-wheels_ratio)*base_thrust
                wheels_torque = maxAA * wheels_ratio * MoI
            else:
                base_thrust = 0
                wheels_torque = engine.maxThrust*lever*2
                MoI = wheels_torque / maxAA
            atc = ATC(engine, lever, MoI,
                      at_pid, av_pid,
                      base_thrust,
                      tune_steering2, wheels_torque)
            return atc.simulate_static_attitude(error, clampL(error*2, 60))

        def simAngle(eng, AA, base_thrust, wheels_ratio, *angles):
            cols = len(angles)
            for c, ang in enumerate(angles):
                ATC.analyze_results(cols, c+1, *[simAA(ang, aa, eng, base_thrust, wheels_ratio) for aa in AA])
            fig = plt.gcf()
            fig.canvas.set_window_title(datetime.strftime(datetime.now(), '%H:%M:%S'))
            plt_show_maxed()

        wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
        LV_T30 = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T30/liquidEngineLV-T30.cfg'))

        wheesly.acceleration /= 2
        wheesly.deceleration /= 2
        # simAngle(wheesly, (0.2, 1, 2, 9, 19), 0.7, 0.01, 45, 15, 3)
        simAngle(wheesly, AAs, 0.7, wheels_ratio, *angles)
        # simAngle(wheesly, (0.5, 0.7, 0.9), 0.7, 0.02, 45, 15, 3)
        # simAngle(wheesly, (0.5, 1, 2, 5, 10, 19), 0.7, 0.02, 45, 15, 3)
        # simAngle(LV_T30, (0.009, 0.03, 0.1, 0.9, 1, 2, 10, 20), 1, 0.02, 175, 60, 15, 3)
        # simAngle(LV_T30, (0.009, 0.03, 0.1), 1, 0.02, 175, 60, 15, 3)

    else:
        print "No %d stage" % stage
