import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from common import dt, clampL, clampH, clamp, PID, PID2, plt_show_maxed, color_grad, Filter, PID3, lerp, clamp01
from Engine import Engine
from Sandbox import Sandbox

drag = 0.005

class HSC(Sandbox):
    def __init__(self, engine, pid, mass, turn_time, on_update=None):
        self.angle = 0
        self.error = 0
        self.time = 0
        self.accel = 0
        self.turn_time = turn_time
        self.turn_speed = 1.0/turn_time*np.log(10)
        self.PID = pid
        self.mass = mass
        self.mg = mass*self.G
        self.engine = engine.clone()
        self.engine.maxThrust *= 4
        self.base_limit = self.mg/self.engine.maxThrust
        assert self.base_limit < 1, 'Ship mass is too great'
        self.engine.limit = self.base_limit
        self.engine.thrust = self.engine.maxThrust*self.engine.limit
        self.PID.max = np.sqrt(1-self.base_limit**2)
        self.on_update = on_update

    def update(self):
        if self.on_update is not None:
            self.on_update(self)
        else:
            self.PID.update(abs(self.error))
        self.angle = lerp(self.angle, np.arctan2(self.PID.action*np.sign(self.error), 1), self.turn_speed*dt)
        self.engine.limit = clamp01(self.base_limit/abs(np.cos(self.angle)))
        self.engine.update()
        self.accel = self.engine.thrust*np.sin(self.angle)/self.mass + (np.random.rand()-0.5)*1e-3
        self.error -= self.accel*dt

    def simulate_constant_speed(self, start_error, end_time, end_on_zero=False):
        self.error = start_error
        self.angle = 0
        self.time = 0
        self.accel = 0
        time = [0]
        error = [self.error]
        action = [self.PID.action*np.sign(self.error)]
        angle = [self.angle/np.pi*180]
        zero_stats = None
        while self.time < end_time:
            self.time += dt
            self.update()
            if not zero_stats and (self.error < 0.01 or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
            error.append(self.error)
            action.append(self.PID.action*np.sign(self.error))
            angle.append(self.angle/np.pi*180)
            time.append(self.time)
        if not zero_stats:
            zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
        return ('dSpd [TT %.2f]' % self.turn_time, time,
                np.fromiter(error, float), action,
                ((angle, 'angle'),), zero_stats)

    def simulate_linear_speed(self, start_error, error_change_rate, end_time, end_on_zero=False):
        self.error = start_error
        self.angle = 0
        self.time = 0
        self.accel = 0
        time = [0]
        error = [self.error]
        action = [self.PID.action * np.sign(self.error)]
        angle = [self.angle/np.pi*180]
        zero_stats = None
        while self.time < end_time:
            self.time += dt
            self.error += error_change_rate*dt
            self.update()
            if not zero_stats and (self.error < 0.01 or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
            error.append(self.error)
            action.append(self.PID.action * np.sign(self.error))
            angle.append(self.angle/np.pi*180)
            time.append(self.time)
        if not zero_stats:
            zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
        return ('dSpd [TT %.2f]' % self.turn_time, time,
                np.fromiter(error, float), action,
                ((angle, 'angle'),), zero_stats)

    def simulate_random_speed(self, start_error, error_change_rate, error_change_time, end_time, end_on_zero=False):
        time_to_change = error_change_time
        self.error = start_error+error_change_rate * (np.random.rand() - 0.5) * 2
        self.angle = 0
        self.time = 0
        self.accel = 0
        time = [0]
        error = [self.error]
        action = [self.PID.action * np.sign(self.error)]
        angle = [self.angle/np.pi*180]
        zero_stats = None
        while self.time < end_time:
            self.time += dt
            time_to_change -= dt * np.random.rand()
            if time_to_change < 0:
                self.error += error_change_rate * (np.random.rand() - 0.5) * 2
                time_to_change = error_change_time
            self.update()
            if not zero_stats and (self.error < 0.01 or self.error*error[-1] < 0):
                zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
            error.append(self.error)
            action.append(self.PID.action * np.sign(self.error))
            angle.append(self.angle/np.pi*180)
            time.append(self.time)
        if not zero_stats:
            zero_stats = self.ZeroStats(self.time, abs(self.accel), 'accel', 'm/s2')
        return ('dSpd [TT %.2f]' % self.turn_time, time,
                np.fromiter(error, float), action,
                ((angle, 'angle'),), zero_stats)


gamedir = u'/media/user/Lir\'s/allis/AT_KSP_Plugins/KSP-test/'
gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
game = u'KSP_test_1.3'
gamedata = u'GameData'


def gamefile(filename): return os.path.join(gamedir, game, filename)


def datafile(filename): return os.path.join(gamedir, game, gamedata, filename)

if __name__ == '__main__':

    def tune_pid(hsc):
        """
        :param hsc: horizontal speed controller
        :type hsc: HSC
        """
        hsc.PID.P = 0.1/hsc.turn_time/(1+abs(hsc.accel))
        hsc.PID.D = 0.03*hsc.turn_time #*(1-clampH(abs(hsc.error)/hsc.turn_time*0.5, 1))
        hsc.PID.update2(abs(hsc.error), -hsc.accel)
        print ('time %f, TT %f, err %f, angle %f, accel %f\nPID %s' %
               (hsc.time, hsc.turn_time, hsc.error, hsc.angle, hsc.accel, hsc.PID))

    mass = 30
    TT = 0.5, 1.5, 3, 6,
    errors = 3, 15, 85,

    def simTurnTime(error, mass, engine, turn_time, error_rate=0, error_time=0):
        pid = PID3(0.05, 0.0, 0.2, 0, 1, 0.1)
        hsc = HSC(engine, pid, mass, turn_time, tune_pid)
        if error_rate > 0:
            if error_time > 0:
                return hsc.simulate_random_speed(error, error_rate, error_time, clampL(error * 2, 60))
            return hsc.simulate_linear_speed(error, error_rate, clampL(error * 2, 60))
        return hsc.simulate_constant_speed(error, clampL(error * 2, 60))

    def simAngle(eng, mass, TT, error_rate, error_time, errors):
        if error_rate > 0:
            Sandbox.analyze_results(1, 1, *[simTurnTime(0, mass, eng, tt, error_rate, error_time) for tt in TT])
        else:
            cols = len(errors)
            for c, error in enumerate(errors):
                Sandbox.analyze_results(cols, c + 1, *[simTurnTime(error, mass, eng, tt) for tt in TT])
        fig = plt.gcf()
        fig.canvas.set_window_title(datetime.strftime(datetime.now(), '%H:%M:%S'))
        plt_show_maxed()

    wheesly = Engine.from_file(datafile('Squad/Parts/Engine/jetEngines/jetEngineBasic.cfg'))
    LV_T30 = Engine.from_file(datafile('Squad/Parts/Engine/liquidEngineLV-T30/liquidEngineLV-T30.cfg'))

    # np.random.seed(1)
    # wheesly.acceleration /= 2
    # wheesly.deceleration /= 2
    simAngle(wheesly, mass, TT, 1, 0, errors)
    # simAngle(LV_T30, mass, TT, 0, 2, errors)
