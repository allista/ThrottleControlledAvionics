from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count

from scipy.optimize import curve_fit

from common import clampL, clampH, clamp01, lerp, PID, PID2, PID3, dt, center_deg, plt_show_maxed

if __name__ == '__main__':
    rad2deg = 180/np.pi
    bearing_pid = PID(
            1, 0.0, 0.0,
            0, np.pi*10
    )

    av_pid = PID3(
            1.0, 0.1, 0,
            -1.0, 1.0,
            3 * dt,
    )


    def timelines(t, *lines):
        n = len(lines)
        ax1 = None
        for i, line in enumerate(lines):
            ax = plt.subplot(n, 1, i + 1, sharex=ax1)
            plt.plot(t, line)
            plt.grid(b=False, which='major', axis='x', color='g', linestyle='-')
            plt.grid(b=False, which='minor', axis='x', color='0.15', linestyle='--')
            plt.grid(b=False, which='major', axis='y', color='g', linestyle='-')
            if ax1 is None:
                ax1 = ax
        plt_show_maxed()


    def sym(angle, maxAA, aaF, max_t, draw=False):
        time = [0]
        angle = [angle]
        av = [0]
        needed_av = [0]
        steering = [0]
        bearing_pid.reset()
        av_pid.reset()
        tune_pids(maxAA, aaF)
        while time[-1] < max_t:
            bearing_action = bearing_pid.update(abs(angle[-1]) / 180)*np.sign(angle[-1])
            needed_av.append(bearing_action)
            steering.append(av_pid.update(bearing_action - av[-1]))
            av.append(av[-1] + av_pid.action * maxAA * dt)
            angle.append(center_deg(angle[-1] - av[-1] * dt * rad2deg))
            time.append(time[-1] + dt)
            # if draw:
                # if abs(angle[-1]) < 0.01 and abs(av[-1]) < 0.01: break
            if not draw and angle[-1] < 0: break
        if draw: timelines(time,
                           np.array(needed_av, dtype=float)*rad2deg,
                           steering,
                           np.array(av, dtype=float)*rad2deg, angle)
        else:
            if abs(angle[-1] < 0.1) and abs(av[-1]) < 1e-3:
                return time[-1]
            return -1


    def optimize((maxAA, max_aaF)):
        print('Optimizing for MaxAA: {}'.format(maxAA))
        best = max_aaF
        best_time = -1
        for aaF in np.linspace(0, max_aaF, 100):
            time = sym(180, maxAA, aaF, 120)
            if time > 0 and (best_time < 0 or time < best_time):
                best = aaF
        return best


    def AAf(aa, a, b, c): return a / (aa ** c + b)


    def fit_AAf(minAA, maxAA, max_aaF, tune_pids):
        mAA = np.arange(minAA, maxAA+0.01, 0.1)
        work = zip(mAA, [max_aaF]*300)
        pool = Pool(cpu_count())
        aaF = pool.map(optimize, work)
        try:
            popt, pcov = curve_fit(AAf, mAA, aaF)
            print(bearing_pid)
            print(('AAf_a = %f\n'
                   'AAf_b = %f\n'
                   'AAf_c = %f\n') % tuple(popt))
            print('['+', '.join('%f' % c for c in popt)+']')
            plt.plot(mAA, aaF)
            plt.plot(mAA, AAf(mAA, *popt))
        except Exception as er:
            print(str(er))
            plt.plot(mAA, aaF)
        plt.show()


    def tune_pids(maxAA, aaF):
        bearing_pid.P = maxAA ** 0.5
        av_pid.P = aaF

    # fit_AAf(2, 10, 50, tune_pids)

    AAf_coef = [1.701136, 0.027771, 0.733734]
    # AAf_coef = [100.786887, 0.009986, 1.013867]

    def plot(angle, maxAA, max_t):
        aaF = min(AAf(maxAA, *AAf_coef), 5)
        sym(angle, maxAA, aaF, max_t, True)


    plot(15, 0.2, 60)
