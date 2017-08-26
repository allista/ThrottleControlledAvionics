import numpy as np
import matplotlib.pyplot as plt

from common import color_grad


class Sandbox(object):
    G = 9.80665
    twoPi = np.pi * 2
    tenth_deg = 0.1 / 180 * np.pi
    rad2deg = 180/np.pi

    class ZeroStats(object):
        def __init__(self, time, speed, desc, units):
            self.time = time
            self.desc = desc or 'Speed'
            self.speed = speed
            self.units = units
            self.metric = self.time+self.speed*100

        def __str__(self):
            return ('Zero at: %f s\n'
                    '%s: %f %s' % (self.time, self.desc, self.speed, self.units))

    @staticmethod
    def _draw_grid(color):
        plt.grid(b=False, which='major', axis='x', color=color, linestyle='-')
        plt.grid(b=False, which='minor', axis='x', color='0.15', linestyle='--')
        plt.grid(b=False, which='major', axis='y', color=color, linestyle='-')

    @classmethod
    def analyze_results(cls, cols, col, *results):
        colors = color_grad(len(results)+1)
        gcolor = colors[-1]
        for i, result in enumerate(results):
            name, time, error, action, addons, zero_stats = result
            if zero_stats:
                print '\n%s\n' % str(zero_stats)
            print '=' * 80
            nplots = 2+len(addons)
            ax = plt.subplot(nplots, cols, col)
            plt.plot(time, error, label=name, color=colors[i])
            cls._draw_grid(gcolor)
            plt.ylabel('error')
            plt.legend()
            plt.subplot(nplots, cols, cols+col, sharex=ax)
            plt.plot(time, action, color=colors[i])
            cls._draw_grid(gcolor)
            plt.ylabel('action')
            if addons:
                for curve, ylab in addons:
                    plt.plot(time, curve, color=colors[i])
                    cls._draw_grid(gcolor)
                    plt.ylabel(ylab)
            plt.xlabel('time')

