from itertools import cycle
import argparse
import matplotlib.pyplot as plt
import os

import numpy as np

from analyze_csv import loadCSV


def load(filename):
    return loadCSV(filename,
                   columns=('time',
                            'rx', 'ry',
                            #                          'vx','vy',
                            #                          'rvx', 'rvy',
                            #                          'Tx', 'Ty',
                            #                          'alt',
                            #                          'mass',
                            #                          'throttle',
                            #'r1x', 'r1y',
                            #                          'nVx', 'nVy'
                            ))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='+')
    args = parser.parse_args()
    fi = np.linspace(0, np.pi * 2, 360)
    x = np.cos(fi) * 600000
    y = np.sin(fi) * 600000
    plt.plot(x, y, 'g-')
    colors = cycle('brcm')
    for filename in args.files:
        if os.path.isfile(filename):
            df = load(filename)
            print(df)
#            tx = np.zeros(len(df.r1x)*2)
#            ty = np.zeros(len(tx))
#            tx[0::2] = df.r1x
#            ty[0::2] = df.r1y
#            plt.plot(tx, ty, 'r-')
            plt.plot(df.rx, df.ry, next(colors))
            # plt.plot(df.time, np.sqrt(df.ry**2 + df.rx**2))
        #    plt.quiver(df.rx, df.ry, df.nVx, df.nVy, scale=0.5, scale_units='xy', units='xy', width=10)
        #    plt.quiver(df.rx, df.ry, df.Tx, df.Ty, scale=0.05, scale_units='xy', units='xy', width=5)
    plt.axis('equal')
    plt.show()
