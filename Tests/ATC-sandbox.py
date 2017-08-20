import os
import csv
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from common import dt, clampL, clampH, clamp, PID, PID2, plt_show_maxed, color_grad, Filter, fit_plot, SimpleKalman, \
    PID3
from Engine import Engine

class ATC(object):
    twoPi = np.pi * 2
    tenth_deg = 0.1/180*np.pi

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
        self.time = 0
        self.AV = 0
        self.AA = 0

    @property
    def errorF(self):
        return abs(self.error/np.pi)

    def reset(self):
        self.AV = 0
        self.AA = 0
        self.engineF.limit = self.base_level
        self.engineF.thrust = self.engineF.maxThrust * self.base_level
        self.engineR.limit = self.base_level
        self.engineR.thrust = self.engineF.maxThrust * self.base_level

    def _error1(self, alpha):
        return 1-alpha + np.random.rand()*alpha*2

    def _error2(self, alpha):
        return (0.5-np.random.rand())*alpha*2

    def updateAV(self):
        action = self.avPID.action
        if self.avPID.action > 0:
            self.engineF.limit = min(self.base_level + action, 1)
            self.engineR.limit = max(self.base_level - action, 0)
        else:
            self.engineR.limit = min(self.base_level - action, 1)
            self.engineF.limit = max(self.base_level + action, 0)
        self.engineF.update()
        self.engineR.update()
        self.AA = ((self.engineF.torque - self.engineR.torque) + self.wheels*self.avPID.action) / self.MoI
        self.AV += self.AA * dt + (np.random.rand()-0.5)*1e-3

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

    def simulate_constant_angular_velocity(self, needed_av, end_time, end_on_zero=False):
        self.time = 0
        time = [0]
        error = [needed_av]
        action = [self.avPID.action]
        thrust = [(self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            av_error = needed_av-self.AV
            if self.on_update is not None:
                self.on_update(self)
            else:
                self.avPID.update(av_error)
            self.updateAV()
            if not zero_stats and (av_error < self.tenth_deg or av_error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AA), 'AA', 'rad/s2')
            error.append(av_error)
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(self.time)
            if zero_stats and end_on_zero: break
        return 'dAV [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.avPID.action]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(self.time)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    def simulate_linear_attitude(self, start_error, error_change_rate, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        error_change_rate *= np.pi/180.0
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            self.error += error_change_rate * dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(self.time)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)/np.pi*180, action, thrust, zero_stats

    def simulate_random_attitude(self, start_error, error_change_rate, error_change_time, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        error_change_rate *= np.pi/180.0
        time_to_change = error_change_time
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
        thrust = [self.engineF.limit]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            time_to_change -= dt*np.random.rand()
            if time_to_change < 0:
                self.error += error_change_rate * (np.random.rand()-0.5) * 2
                time_to_change = error_change_time
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV+self.AA*dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            thrust.append((self.engineF.thrust-self.engineR.thrust) / self.engineF.maxThrust)
            time.append(self.time)
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
            ax = plt.subplot(3, cols, col)
            plt.plot(time, error, label=name, color=colors[i])
            plt.ylabel('error (deg)')
            plt.legend()
            plt.subplot(3, cols, cols+col, sharex=ax)
            plt.plot(time, action, color=colors[i])
            plt.ylabel('action')
            plt.subplot(3, cols, cols*2+col, sharex=ax)
            plt.plot(time, thrust, color=colors[i])
            plt.xlabel('time')
            plt.ylabel('thrust')


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
        # compute coefficients of attitude PID
        atP_iErrf = clampL(iErrf - cfg.atP_ErrThreshold, 0) ** cfg.atP_ErrCurve
        if atc.MaxAA >= 1:
            atc.atPID.P = clampH(1
                                 + cfg.atP_HighAA_Scale * atc.MaxAA ** cfg.atP_HighAA_Curve
                                 + atP_iErrf, cfg.atP_HighAA_Max)
            atc.atPID.D = cfg.atD_HighAA_Scale * imaxAA ** cfg.atD_HighAA_Curve * clampH(iErrf + abs(AM), 1.2)
        else:
            atc.atPID.P = (1
                           + cfg.atP_LowAA_Scale * atc.MaxAA ** cfg.atP_LowAA_Curve
                           + atP_iErrf)
            atc.atPID.D = cfg.atD_LowAA_Scale * imaxAA ** cfg.atD_LowAA_Curve * clampH(iErrf + abs(AM), 1.2)
        atI_iErrf = clampL(iErrf - cfg.atI_ErrThreshold, 0)
        overshot = atc.AV * atc.error < 0
        if atI_iErrf <= 0 or overshot:
            atc.atPID.I = 0
            atc.atPID.ierror = 0
        else:
            atI_iErrf = atI_iErrf ** cfg.atI_ErrCurve
            atc.atPID.I = (cfg.atI_Scale * atc.MaxAA * atI_iErrf /
                           (1 + clampL(atc.AV * np.sign(atc.error), 0) * cfg.atI_AV_Scale * atI_iErrf))
        # compute coefficients of angular velocity PID
        atc.avPID.P = clampL(cfg.avP_MaxAA_Intersect -
                             cfg.avP_MaxAA_Inclination * atc.MaxAA ** cfg.avP_MaxAA_Curve,
                             cfg.avP_Min)
        atc.avPID.I = cfg.avI_Scale * atc.avPID.P
        atc.avPID.D = 0
        AV = atc.AV
        update_pids(atc, AV)

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
        avP_A = 100
        avP_B = 0.04
        avP_C = -100
        avP_D = 0.7

        avD_A = 0.65
        avD_B = -0.01
        avD_C = 0.5
        avD_D = 0.7

        avI_Scale = 0.03

    avFilter = Filter(0.8)
    avFilter.setTau(3*dt)

    def tune_steering_mixed(atc, iErrf, imaxAA, AM):
        if atc.InstantRatio > 0.3:
            tune_steering_fast(MixedConfig3plus, atc, iErrf, imaxAA, AM)
        else:
            # if atc.MaxAA >= 1:
            #     atc.atPID.P = 20
            #     atc.atPID.I = 0
            #     atc.atPID.D = 0
            #     atc.avPID.P = 1/clampL(abs(AM)/atc.MaxAA**0.2/12, 1)
            #     atc.avPID.I = 0.002
            #     atc.avPID.D = 1.4/atc.MaxAA**0.3
            # else:
            #     atc.atPID.P = 40/clampL(abs(AM)/2, 1)
            #     atc.atPID.I = 0
            #     atc.atPID.D = 0
            #     atc.avPID.P = 1/clampL(abs(AM)/12, 1)
            #     atc.avPID.I = 0.001
            #     atc.avPID.D = 1.4/atc.MaxAA**0.3

            # clampH(abs(40*(abs(atc.avPID.perror)+abs(atc.error/np.pi)))**6, 1)
            noise_scale = clamp((50 * (abs(atc.AV) + atc.errorF)) ** 0.6, 0.001, 1)

            atc.atPID.P = 1#clamp(0.5*atc.MaxAA**0.5, 0.0001, 1)
            atc.atPID.I = 0
            atc.atPID.D = 0

            atc.avPID.P = ((MixedConfig.avP_A / (atc.InstantRatio ** MixedConfig.avP_D + MixedConfig.avP_B) +
                           MixedConfig.avP_C)) / clampL(abs(AM), 1) / atc.MaxAA * noise_scale
            atc.avPID.D = ((MixedConfig.avD_A / (atc.InstantRatio ** MixedConfig.avD_D + MixedConfig.avD_B) +
                            MixedConfig.avD_C)) / atc.MaxAA * noise_scale
            atc.avPID.I = MixedConfig.avI_Scale * clampH(atc.MaxAA, 1) * noise_scale

            # atc.avPID.P = (1 + 8 / atc.MaxAA) * noise_scale
            # atc.avPID.D = clampH((clampL(abs(AM) / atc.MaxAA, 1) ** 0.1 - 1), 2) * noise_scale


            # avFilter.ratio = clamp((abs(atc.avPID.perror)+abs(atc.error/np.pi))/atc.InstantRatio*50, 0.1, 1)
            # AV = avFilter.EWA(atc.AV)
            # print avFilter.ratio
            AV = atc.AV
            update_pids(atc, AV)

    IRt = [(0.299, 0.5),
           (0.2, 0.35),
           (0.1, 0.2),
           (0.05, 0.1),
           (0.01, 0.3),
           ]

    wheels_ratio = 0.2
    AAs = 0.3, 0.7, 0.9, 1, 1.9, 3, 9, 20
    # AAs = 1, 1.9, 3, 9
    AAs = 2,
    angles = 85, 25, 3

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
        atc.atPID.P = 1
        atc.atPID.I = 0
        atc.atPID.D = 0
        slowF = (1 + SlowConfig.SlowTorqueF / max(atc.engineF.acceleration, atc.engineF.deceleration))
        noise_scale = clamp(abs(50 * (abs(atc.avPID.perror) + abs(atc.error / np.pi))) ** 0.5, 0.01, 1)
        if atc.MaxAA >= 1:
            atc.avPID.P = SlowConfig.avP_HighAA_Scale/slowF * noise_scale
            atc.avPID.D = clampL(SlowConfig.avD_HighAA_Intersect - SlowConfig.avD_HighAA_Inclination * atc.MaxAA,
                                 SlowConfig.avD_HighAA_Max) * noise_scale
        else:
            atc.avPID.P = SlowConfig.avP_LowAA_Scale/slowF * noise_scale
            atc.avPID.D = (SlowConfig.avD_LowAA_Intersect - SlowConfig.avD_LowAA_Inclination * atc.MaxAA) * noise_scale
        atc.avPID.I = SlowConfig.avI_Scale*clampH(atc.MaxAA, 1) * noise_scale
        # avFilter.ratio = clamp(abs(atc.AV)+abs(atc.error/np.pi)*500, 0.01, 1)
        # AV = avFilter.EWA(atc.AV)
        AV = atc.AV
        update_pids(atc, AV)

    def update_pids(atc, AV):
        atc.atPID.update2(abs(atc.error), -AV)
        avErr = abs(atc.atPID.action) * np.sign(atc.error) - AV
        atc.avPID.update(avErr)
        atc.avPID.action = avFilter.EWA(clamp(atc.avPID.action, -1, 1))

    def tune_steering(atc):
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
        print ('time %f, atc.MaxAA %f, err %f, av %f, am %f, iErr %f\natPID %s\navPID %s' %
               (atc.time, atc.MaxAA, atc.error / np.pi * 180, atc.AV / np.pi * 180, AM, iErrf, atc.atPID, atc.avPID))

    lever = 4

    def simAA(error, maxAA, engine, base_thrust, wheels_ratio, error_rate=0, error_time=0):
        at_pid = PID2(1, 0.0, 1, 0, np.pi*10)
        av_pid = PID3(1, 0.0, 1, -1, 1, 3*dt)
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
                  tune_steering, wheels_torque)
        if error_rate > 0:
            if error_time > 0:
                return atc.simulate_random_attitude(error, error_rate, error_time, clampL(error * 2, 60))
            return atc.simulate_linear_attitude(error, error_rate, clampL(error * 2, 60))
        return atc.simulate_static_attitude(error, clampL(error * 2, 60))

    def simAngle(eng, AA, base_thrust, wheels_ratio, error_rate, error_time, angles):
        if error_rate > 0:
            ATC.analyze_results(1, 1, *[simAA(0, aa, eng, base_thrust, wheels_ratio, error_rate, error_time) for aa in AA])
        else:
            cols = len(angles)
            for c, ang in enumerate(angles):
                ATC.analyze_results(cols, c + 1, *[simAA(ang, aa, eng, base_thrust, wheels_ratio) for aa in AA])
        fig = plt.gcf()
        fig.canvas.set_window_title(datetime.strftime(datetime.now(), '%H:%M:%S'))
        plt_show_maxed()

    wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
    LV_T30 = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T30/liquidEngineLV-T30.cfg'))

    # np.random.seed(1)
    wheesly.acceleration /= 2
    wheesly.deceleration /= 2
    # simAngle(wheesly, (0.2, 1, 2, 9, 19), 0.7, 0.01, 0, (45, 15, 3))
    simAngle(wheesly, AAs, 0.7, wheels_ratio, 10, 2, angles)
    # simAngle(wheesly, (0.5, 0.7, 0.9), 0.7, 0.02, 0, (45, 15, 3))
    # simAngle(wheesly, (0.5, 1, 2, 5, 10, 19), 0.7, 0.02, 0, (45, 15, 3))
    # simAngle(LV_T30, (0.009, 0.03, 0.1, 0.9, 1, 2, 10, 20), 1, 0.02, 0, (175, 60, 15, 3))
    # simAngle(LV_T30, (0.009, 0.03, 0.1), 1, 0.02, 0, (175, 60, 15, 3))
    # simAngle(LV_T30, (0.009, 0.03, 0.1, 0.3, 0.7, 0.9), 1, 0.02, 0, (175, 60, 15, 3))
