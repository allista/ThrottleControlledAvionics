import os
import matplotlib.pyplot as plt
import numpy as np


from analyze_csv import loadCSV


if __name__ == '__main__':
    df = loadCSV(os.path.expanduser('~/ToOrbitSim.csv'),
                 columns=('time',
                          'rx','ry',
                          'vx','vy',
                          'rvx', 'rvy',
                          'Tx', 'Ty',
                          'alt',
                          'mass',
                          'throttle',
                          'r1x', 'r1y',
                          'nVx', 'nVy'
                          ))
    print(df)
    fi = np.linspace(0, np.pi*2, 360)
    x = np.cos(fi)*600000
    y = np.sin(fi)*600000
    plt.plot(x, y, 'g-')
    plt.plot((0, df.r1x[0]), (0, df.r1y[0]), 'r-')
    plt.plot(df.rx, df.ry)
    plt.quiver(df.rx, df.ry, df.nVx, df.nVy, scale=0.5, scale_units='xy', units='xy', width=10)
    plt.quiver(df.rx, df.ry, df.Tx, df.Ty, scale=0.05, scale_units='xy', units='xy', width=5)
    plt.axis('equal')
    plt.show()
