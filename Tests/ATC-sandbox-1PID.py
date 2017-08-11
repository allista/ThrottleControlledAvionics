import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from common import dt, clampL, clampH, clamp, lerp, PID, PID2, plt_show_maxed, color_grad, fit_plot, Filter
from Engine import Engine


class ATC(object):
    twoPi = np.pi * 2
    tenth_deg = 0.1 / 180 * np.pi

    def __init__(self, engine, lever, MoI, atPID, base_level, on_update=None, wheels_torque=0):
        self.error = 0
        self.atPID = atPID
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
        if self.atPID.action > 0:
            self.engineF.limit = self.base_level
            self.engineR.limit = max(self.base_level - self.atPID.action, 0)
        else:
            self.engineR.limit = self.base_level
            self.engineF.limit = max(self.base_level + self.atPID.action, 0)
        self.engineF.update()
        self.engineR.update()
        self.AA = (self.engineF.torque - self.engineR.torque + self.wheels*self.atPID.action) / self.MoI
        self.AV += self.AA * dt + (np.random.rand()-0.5)*1e-3

    def update(self):
        if self.on_update is not None:
            self.on_update(self)
        else:
            self.atPID.update(self.error)
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

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        t = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
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
            action.append(self.atPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(t)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    def simulate_linear_attitude(self, start_error, error_change_rate, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        error_change_rate *= np.pi/180.0
        t = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while t < end_time:
            t += dt
            self.error += error_change_rate * dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(t, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.atPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(t)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    def optimize_at_PID(self, start_error, av_threshold, step, end_time, P, I, D):
        best_pid = None
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                for d in np.linspace(D[0], D[1], step):
                    self.atPID.setPID(p,i,d)
                    name, time, error, limit, thrust, zero_stats = \
                        self.simulate_static_attitude(start_error, end_time, True)
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


gamedir = u'/media/user/Lir\'s/allis/AT_KSP_Plugins/KSP-test/'
gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
game = u'KSP_test_1.3'
gamedata = u'GameData'

def gamefile(filename): return os.path.join(gamedir, game, filename)
def datafile(filename): return os.path.join(gamedir, game, gamedata, filename)

if __name__ == '__main__':
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
        :param atc: Attitude Control model
        :type atc: ATC
        :param iErrf: inverse error factor
        :type iErrf: float
        :param imaxAA: inverse maxAA
        :type imaxAA: float
        :param AM: angular momentum
        :type AM: float
        """
        # for PID2
        atc.atPID.P = 1 + 50 * iErrf ** 2.2
        atc.atPID.D = 20 * imaxAA ** 0.5 * iErrf ** 0.2
        atc.atPID.I = 0.9 * imaxAA ** 0.3
        update_pids(cfg, atc, iErrf)

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
        avP_A = 50
        avP_B = 0.04
        avP_C = -50
        avP_D = 0.7

        avD_A = 0.65
        avD_B = -0.01
        avD_C = 0.5
        avD_D = 0.7

        avI_Scale = 0.03

    def tune_steering_mixed(atc, iErrf, imaxAA, AM):
        if atc.InstantRatio > 0.1:
            atc.atPID.P = 1 + 50 * iErrf ** 2.2 * atc.InstantRatio**0.8
            atc.atPID.D = 20 * imaxAA ** 0.5 * iErrf ** 0.3
            atc.atPID.I = 2 * imaxAA ** 0.3 * atc.InstantRatio
        else:
            if atc.MaxAA >= 1:
                atc.atPID.P = (1 + 1*atc.MaxAA + 9 * iErrf ** 2)*(10*atc.InstantRatio+0.3)
                atc.atPID.D = 20+clampL((11-atc.MaxAA**2)*3, 0)
                atc.atPID.I = 0.1 + 0.7*atc.MaxAA**0.5
            else:
                atc.atPID.P = (1 + 3 * atc.MaxAA + 7 * iErrf ** 2) * (10 * atc.InstantRatio + 0.3)
                atc.atPID.D = 20 + clampL((11 - atc.MaxAA ** 0.5) * 3, 0)
                atc.atPID.I = 0.1 + 0.7 * atc.MaxAA ** 0.5
        update_pids(MixedConfig, atc, iErrf)

    # 0.09, 1
    # data = [(0.07, 1),
    #         (0.05, 0.8),
    #         (0.04, 0.7)]
    # print fit_plot(data, lambda x, a,b,c: a*x**b+c)


    wheels_ratio = 0.05
    AAs = 0.3, 0.7, 0.9, 1.2, 3, 7
    AAs = 1, 1.9, 3, 9
    angles = 85, 15, 3

    class SlowConfig(object):
        avP_HighAA_Scale = 5
        avD_HighAA_Intersect = 10
        avD_HighAA_Inclination = 2
        avD_HighAA_Max = 2

        avP_LowAA_Scale = 8
        avD_LowAA_Intersect = 25
        avD_LowAA_Inclination = 10

        avI_Scale = 0.005

        SlowTorqueF = 0.2

    def tune_steering_slow(atc, iErrf, imaxAA, AM):
        slowF = (1 + SlowConfig.SlowTorqueF / max(atc.engineF.acceleration, atc.engineF.deceleration))
        if atc.MaxAA >= 1:
            atc.atPID.P = 8
            atc.atPID.D = 50
            atc.atPID.I = 0#.1 + 0.7*atc.MaxAA**0.5
        else:
            atc.atPID.P = (1 + 3 * atc.MaxAA + 7 * iErrf ** 2)
            atc.atPID.D = 20 + clampL((11 - atc.MaxAA ** 0.5) * 3, 0)
            atc.atPID.I = 0.1 + 0.7 * atc.MaxAA ** 0.5
        update_pids(MixedConfig, atc, iErrf)


    avFilter = Filter(0.1)
    def update_pids(cfg, atc, iErrf):
        # avFilter.ratio = clamp(abs(atc.AV) + abs(atc.error / np.pi), 0.1, 1)
        # AV = avFilter.EWA(atc.AV)
        atc.atPID.update2(atc.error, -atc.AV)
        atc.atPID.action = clamp(atc.atPID.action, -1, 1)

    def tune_steering(atc):
        """
        :param atc: attitude controller
        :type atc: ATC
        """
        iErrf = 1 - abs(atc.error/np.pi)
        imaxAA = 1 / atc.MaxAA
        AM = atc.AV*atc.MoI
        if atc.InstantRatio == 1:
            tune_steering_fast(FastConfig, atc, iErrf, imaxAA, AM)
        elif atc.InstantRatio >= 0.005:
            tune_steering_mixed(atc, iErrf, imaxAA, AM)
        else:
            tune_steering_slow(atc, iErrf, imaxAA, AM)
        print ('atc.MaxAA %f, err %f, av %f, am %f, iErr %f\natPID %s' %
               (atc.MaxAA, atc.error / np.pi * 180, atc.AV / np.pi * 180, AM, iErrf, atc.atPID))

    lever = 4

    def simAA(error, maxAA, engine, base_thrust, wheels_ratio, error_rate=0):
        at_pid = PID2(1, 0.0, 1, -1, 1)
        if wheels_ratio < 1:
            MoI = engine.maxThrust*lever*2/maxAA/(1-wheels_ratio)*base_thrust
            wheels_torque = maxAA * wheels_ratio * MoI
        else:
            base_thrust = 0
            wheels_torque = engine.maxThrust*lever*2
            MoI = wheels_torque / maxAA
        atc = ATC(engine, lever, MoI,
                  at_pid,
                  base_thrust,
                  tune_steering, wheels_torque)
        if error_rate > 0:
            return atc.simulate_linear_attitude(error, error_rate, clampL(error * 2, 60))
        return atc.simulate_static_attitude(error, clampL(error * 2, 60))

    def simAngle(eng, AA, base_thrust, wheels_ratio, error_rate, angles):
        cols = 1 if error_rate > 0 else len(angles)
        if error_rate > 0:
            ATC.analyze_results(cols, 1, *[simAA(0, aa, eng, base_thrust, wheels_ratio, error_rate) for aa in AA])
        else:
            for c, ang in enumerate(angles):
                ATC.analyze_results(cols, c+1, *[simAA(ang, aa, eng, base_thrust, wheels_ratio) for aa in AA])
        fig = plt.gcf()
        fig.canvas.set_window_title(datetime.strftime(datetime.now(), '%H:%M:%S'))
        plt_show_maxed()

    wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
    LV_T30 = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T30/liquidEngineLV-T30.cfg'))

    error_rate = 0
    # wheesly.acceleration /= 5
    # wheesly.deceleration /= 5
    # simAngle(wheesly, (0.2, 1, 2, 9, 19), 0.7, 0.01, error_rate, (45, 15, 3))
    simAngle(wheesly, AAs, 0.7, wheels_ratio, error_rate, angles)
    # simAngle(wheesly, (0.5, 0.7, 0.9), 0.7, 0.02, error_rate, (45, 15, 3))
    # simAngle(wheesly, (0.5, 1, 2, 5, 10, 19), 0.7, 0.02, error_rate, (45, 15, 3))
    simAngle(LV_T30, (0.009, 0.03, 0.1, 0.9, 1, 2, 10, 20), 1, 0.02, error_rate, (175, 60, 15, 3))
    # simAngle(LV_T30, (0.009, 0.03, 0.1, 0.3, 0.7, 0.9), 1, 0.02, error_rate, (175, 60, 15, 3))
    # simAngle(LV_T30, (1, 3, 5, 7, 9), 1, 0.02, error_rate, (175, 60, 15, 3))
