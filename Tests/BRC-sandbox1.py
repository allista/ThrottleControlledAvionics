import matplotlib.pyplot as plt
import os
from datetime import datetime

import numpy as np

from Sandbox import Sandbox
from common import dt, clampL, clampH, clamp, PID2, plt_show_maxed, Filter, PID3


class BRC(Sandbox):
    def __init__(self, wheels_torque, MoI, atPID, avPID, on_update=None):
        self.error = 0
        self.atPID = atPID
        self.avPID = avPID
        self.wheels = wheels_torque
        self.MoI = MoI
        self.MaxAA = self.wheels / self.MoI
        self.on_update = on_update
        self.time = 0
        self.AV = 0
        self.AA = 0

    @property
    def errorF(self):
        return abs(self.error / np.pi)

    def reset(self):
        self.AV = 0
        self.AA = 0

    def _error1(self, alpha):
        return 1 - alpha + np.random.rand() * alpha * 2

    def _error2(self, alpha):
        return (0.5 - np.random.rand()) * alpha * 2

    def updateAV(self):
        self.AA = self.wheels * self.avPID.action / self.MoI
        self.AV += self.AA * dt + (np.random.rand() - 0.5) * 1e-3

    def update(self):
        if self.on_update is not None:
            self.on_update(self)
        else:
            self.atPID.update(abs(self.error))
            self.avPID.update(self.atPID.action * np.sign(self.error) - self.AV)
        self.updateAV()
        self.error -= self.AV * dt
        self.error %= self.twoPi
        if self.error > np.pi: self.error -= self.twoPi
        elif self.error < -np.pi: self.error += self.twoPi

    def simulate_constant_angular_velocity(self, needed_av, end_time, end_on_zero=False):
        self.time = 0
        time = [0]
        error = [needed_av]
        action = [self.avPID.action]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            av_error = needed_av - self.AV
            if self.on_update is not None:
                self.on_update(self)
            else:
                self.avPID.update(av_error)
            self.updateAV()
            if not zero_stats and (av_error < self.tenth_deg or av_error * error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AA), 'AA', 'rad/s2')
            error.append(av_error)
            action.append(self.avPID.action)
            time.append(self.time)
            if zero_stats and end_on_zero: break
        return 'dAV [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)*self.rad2deg, action, (), zero_stats

    def simulate_static_attitude(self, start_error, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.avPID.action]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error * error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV + self.AA * dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            time.append(self.time)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)*self.rad2deg, action, (), zero_stats

    def simulate_linear_attitude(self, start_error, error_change_rate, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        error_change_rate *= np.pi / 180.0
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            self.error += error_change_rate * dt
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error * error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV + self.AA * dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            time.append(self.time)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)*self.rad2deg, action, (), zero_stats

    def simulate_random_attitude(self, start_error, error_change_rate, error_change_time, end_time, end_on_zero=False):
        self.error = start_error / 180.0 * np.pi
        error_change_rate *= np.pi / 180.0
        time_to_change = error_change_time
        self.time = 0
        time = [0]
        error = [self.error]
        action = [self.atPID.action]
        zero_stats = None
        self.reset()
        while self.time < end_time:
            self.time += dt
            time_to_change -= dt * np.random.rand()
            if time_to_change < 0:
                self.error += error_change_rate * (np.random.rand() - 0.5) * 2
                time_to_change = error_change_time
            self.update()
            if not zero_stats and (self.error < self.tenth_deg or self.error * error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.AV + self.AA * dt), 'AV', 'rad/s')
            error.append(self.error)
            action.append(self.avPID.action)
            time.append(self.time)
        return 'dAng [AA %.2f]' % self.MaxAA, time, np.fromiter(error, float)*self.rad2deg, action, (), zero_stats

    def optimize_av_PID(self, needed_av, aa_threshold, step, end_time, P, I, D):
        best_pid = (1, 0, 0)
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                for d in np.linspace(D[0], D[1], step):
                    self.avPID.setPID(p, i, d)
                    name, time, error, limit, thrust, zero_stats = self.simulate_constant_angular_velocity(needed_av,
                                                                                                           end_time,
                                                                                                           True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < aa_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p, i, d)
        if best_stats and best_stats.speed < aa_threshold:
            self.avPID.setPID(*best_pid)
            print 'avPID: %s\n%s\n' % (best_pid, best_stats)
            return best_pid
        return None

    def optimize_av_D(self, needed_av, aa_threshold, step, end_time, D):
        best_pid = (1, 0, 0)
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
        best_pid = (1, 0, 0)
        best_stats = None
        for p in np.linspace(P[0], P[1], step):
            for i in np.linspace(I[0], I[1], step):
                for d in np.linspace(D[0], D[1], step):
                    self.atPID.setPID(p, i, d)
                    name, time, error, limit, thrust, zero_stats = self.simulate_static_attitude(start_error, end_time,
                                                                                                 True)
                    if not zero_stats: continue
                    if not best_stats or zero_stats.speed < av_threshold and zero_stats.metric < best_stats.metric:
                        best_stats = zero_stats
                        best_pid = (p, i, d)
        if best_stats and best_stats.speed < av_threshold:
            self.atPID.setPID(*best_pid)
            print 'atPID: %s\n%s\n' % (best_pid, best_stats)
            return best_pid
        return None


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

        atP_HighAA_Scale = 0.7
        atP_HighAA_Curve = 0.1
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
        :type atc: BRC
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


    AAs = 0.1, 0.3, #0.7, 0.9, 1, 1.9, 3, 9, 20
    angles = -85, -25, -3
    avFilter = Filter(0.8)
    avFilter.setTau(3 * dt)


    def update_pids(atc, AV):
        atc.atPID.update2(abs(atc.error), -np.sign(atc.error)*AV)
        avErr = abs(atc.atPID.action) * np.sign(atc.error) - AV
        atc.avPID.update(avErr)
        atc.avPID.action = avFilter.EWA(clamp(atc.avPID.action, -1, 1))


    def tune_steering(atc):
        """
        :param atc: attitude controller
        :type atc: BRC
        """
        iErrf = 1 - abs(atc.error / np.pi)
        imaxAA = 1 / atc.MaxAA
        AM = atc.AV * atc.MoI
        tune_steering_fast(FastConfig, atc, iErrf, imaxAA, AM)
        print ('time %f, atc.MaxAA %f, err %f, av %f, am %f, iErr %f\natPID %s\navPID %s' %
               (atc.time, atc.MaxAA, atc.error / np.pi * 180, atc.AV / np.pi * 180, AM, iErrf, atc.atPID, atc.avPID))


    def simAA(error, maxAA, error_rate=0, error_time=0):
        at_pid = PID2(1, 0, 0, 0, np.pi * 10)
        av_pid = PID3(1, 0.1, 0, -1, 1, 3 * dt)
        wheels_torque = maxAA
        MoI = 1
        atc = BRC(wheels_torque, MoI,
                  at_pid, av_pid,
                  tune_steering)
        if error_rate > 0:
            if error_time > 0:
                return atc.simulate_random_attitude(error, error_rate, error_time, clampL(error * 2, 60))
            return atc.simulate_linear_attitude(error, error_rate, clampL(error * 2, 60))
        return atc.simulate_static_attitude(error, clampL(error * 2, 60))


    def simAngle(AAs, error_rate, error_time, angles):
        if error_rate > 0:
            BRC.analyze_results(1, 1,
                                *[simAA(0, aa, error_rate, error_time) for aa in AAs])
        else:
            cols = len(angles)
            for c, ang in enumerate(angles):
                BRC.analyze_results(cols, c + 1, *[simAA(ang, aa) for aa in AAs])
        fig = plt.gcf()
        fig.canvas.set_window_title(datetime.strftime(datetime.now(), '%H:%M:%S'))
        plt_show_maxed()


    # np.random.seed(1)
    simAngle(AAs, 0, 0, angles)
