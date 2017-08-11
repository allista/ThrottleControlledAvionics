from __future__ import print_function
from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit

from common import clampL, clampH, clamp01, lerp, PID, PID2, dt, center_deg

if __name__ == '__main__':
    amp = np.pi
    bearing_pid = PID(
            amp, 0.001, 1.0,
            -amp, amp
    )

    av_pid = PID(
        1.0, 0.05, 0,
        -1.0, 1.0
    )

    def timelines(t, *lines):
        n = len(lines)
        for i in range(n):
            plt.subplot(n, 1, i+1)
            plt.plot(t, lines[i])
        plt.show()

    def sym(angle, maxAA, max_t, aaF, draw=False):
        t = [0]
        a = [angle]
        av = [0]
        avn = [0]
        st = [0]
        bearing_pid.reset()
        av_pid.reset()
        bearing_pid.D = aaF
        while t[-1] < max_t:
            avn.append(bearing_pid.update(a[-1]/180))
            st.append(av_pid.update(bearing_pid.action-av[-1]))
            av.append(av[-1]+av_pid.action*maxAA*dt)
            a.append(center_deg(a[-1]-av[-1]*dt*180))
            t.append(t[-1]+dt)
            if draw:
                if abs(a[-1]) < 0.01 and abs(av[-1]) < 0.01: break
            else:
                if a[-1] < 0: break
        if draw: timelines(t, avn, st, av, a)
        else: return a[-1] > 0

    def optimize(maxAA, max_aaF):
        for _aaF in np.linspace(0, max_aaF, 200):
            if sym(180, maxAA, 120, _aaF):
                return _aaF
        return max_aaF

    def AAf(aa, a, b, c): return a / (aa ** c + b)

    def fit_AAf(max_aaF):
        bearing_pid.I = 0
        mAA = np.linspace(1e-3, 15, 300)
        aaF = [optimize(aa, max_aaF) for aa in mAA]
        popt, pcov = curve_fit(AAf, mAA, aaF)
        print(bearing_pid)
        print(('AAf_a = %f\n'
               'AAf_b = %f\n'
               'AAf_c = %f') % tuple(popt))
        print(', '.join('%f' % c for c in popt))
        plt.plot(mAA, aaF)
        plt.plot(mAA, AAf(mAA, *popt))
        plt.show()

    # fit_AAf(50)

    AAf_coef = [1.701136, 0.027771, 0.733734]

    def plot(angle, maxAA, max_t):
        aaF = AAf(maxAA, *AAf_coef)
        print(aaF)
        sym(angle, maxAA, max_t, aaF, True)
        print(bearing_pid)

    plot(180, 10, 15)
