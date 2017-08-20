import numpy as np
import matplotlib.pyplot as plt

from common import color_grad


class Sandbox(object):
    G = 9.80665
    twoPi = np.pi * 2

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

    @classmethod
    def analyze_results(cls, cols, col, *results):
        colors = color_grad(len(results))
        for i, result in enumerate(results):
            name, time, error, action, addons, zero_stats = result
            if zero_stats:
                print '\n%s\n' % str(zero_stats)
            print '=' * 80
            nplots = 2+len(addons)
            ax = plt.subplot(nplots, cols, col)
            plt.plot(time, error, label=name, color=colors[i])
            plt.ylabel('error')
            plt.legend()
            plt.subplot(nplots, cols, cols+col, sharex=ax)
            plt.plot(time, action, color=colors[i])
            plt.ylabel('action')
            plt.subplot(nplots, cols, cols*2+col, sharex=ax)
            if addons:
                for curve, ylab in addons:
                    plt.plot(time, curve, color=colors[i])
                    plt.ylabel(ylab)
            plt.xlabel('time')

