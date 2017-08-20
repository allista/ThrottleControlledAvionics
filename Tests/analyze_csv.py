"""
Created on Jan 8, 2015

@author: Allis Tauri
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import pandas as pd
import os

from common import plt_show_maxed, color_grad


def loadCSV(filename, columns=None, header=None):
    os.chdir(os.path.join(os.environ['HOME'], 'ThrottleControlledAvionics'))
    df = pd.read_csv(filename, header=header, names=columns)
    return df


def drawDF(df, x, columns, colors=None, axes=None):
    from collections import Counter
    if axes is not None:
        num_axes = Counter(axes)
        nrows = max(num_axes.values())
        ncols = len(num_axes.keys())
    X = df[x]
    ax1 = None
    if colors is None: colors = color_grad(len(columns))
    for i, k in enumerate(columns):
        if axes is None:
            plt.plot(X, df[k], label=k, color=colors[i])
        else:
            ax = plt.subplot(nrows, ncols, ncols * (i % nrows) + axes[i], sharex=ax1)
            ax.plot(X, df[k], label=k, color=colors[i])
            plt.ylabel(k)
            if ax1 is None: ax1 = ax
        plt.minorticks_on()
        plt.grid(b=False, which='major', axis='x', color='b', linestyle='-')
        plt.grid(b=False, which='minor', axis='x', color='0.15', linestyle='--')
        plt.grid(b=False, which='major', axis='y', color='b', linestyle='-')
        #     plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.0)
    plt_show_maxed()


def boxplot(df, columns=None):
    cnames = df.keys() if columns is None else columns
    plt.boxplot([df[k] for k in cnames], labels=cnames)
    plt_show_maxed()


def describe(df, columns=None):
    from scipy import stats
    cnames = df.keys() if columns is None else columns
    for k in cnames:
        c = df[k]
        d = stats.describe(c)
        print '%s:' % k
        print '   len:    ', len(c)
        print '   sum:    ', sum(c)
        print '   min-max:', d.minmax
        print '   mean:   ', d.mean
        print '   median: ', np.median(c)
        print '   std:    ', np.std(c)
        print '   sem:    ', stats.sem(c)
        print '   skew:   ', d.skewness
        print '   hist:   '
        h = np.histogram(c, normed=True)
        for i, e in enumerate(h[1][1:]):
            e0 = h[1][i]
            print '[% 8.3f : % 8.3f]: %s' % (e0, e, "#" * int(h[0][i] * 80))
        print ''
    print '\n'


def addL(df):
    """add horizontal coordinate column"""
    L = [0]
    if 'hV' in df:
        for hv in df.hV[:-1]:
            L.append(L[-1] + hv * dt)
    else:
        L = np.arange(0, df.shape[0], 1)
    df['L'] = pd.Series(L, index=df.index)


def analyzeCSV(filename, header, cols=None, x=None, axes=(), region=None):
    df = loadCSV(filename, header)
    if 'name' in df:
        del df['name']
        df.reset_index()
    if 'AltitudeAhead' in df:
        df.AltitudeAhead[df.AltitudeAhead > 10000] = 0
    if 'Alt' in df:
        df = df[df.Alt > 3].reset_index()
    if 'UT' in df:
        df['UT'] -= df.UT[0]
    addL(df)
    # slice by L
    if region:
        region = list(region)
        if len(region) < 2: region.append(None)
        if region[0] == None: region[0] = 0
        if region[1] == None: region[1] = df.L.last
        df = df[(df.L > region[0]) & (df.L <= region[1])]
    #     print df.iloc[500]
    if cols is None:
        cols = list(df.keys())
        if 'L' in cols: cols.remove('L')
        if x in cols: cols.remove(x)
    drawDF(df, 'L' if x is None else x, cols, axes=[1] * len(cols) if axes is () else axes)
    return df


gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
game = u'KSP_test_1.3'

def gamefile(filename): return os.path.join(gamedir, game, filename)


if __name__ == '__main__':
    analyzeCSV(
            gamefile('Tardigrade.AttitudeControl.csv'),
            (
                'Alt',
                'Ex', 'Ey', 'Ez',
                'Sx', 'Sy', 'Sz',
                'AVx', 'AVy', 'AVz',
                'AMx', 'AMy', 'AMz',
                'INx', 'INy', 'INz',
                'Ax', 'Ay', 'Az',
                'Px', 'Py', 'Pz',
                'Ix', 'Iy', 'Iz',
                'Dx', 'Dy', 'Dz',
                'AAx', 'AAy', 'AAz',
                'PIfx', 'PIfy', 'PIfz',
                'AAfx', 'AAfy', 'AAfz',
                'SLx', 'SLy', 'SLz',
            ),
            (
                'Ex', 'Ey', 'Ez',
                'Ax', 'Ay', 'Az',
                #'Sx', 'Sy', 'Sz',
                'AVx', 'AVy', 'AVz',
                #'AMx', 'AMy', 'AMz',
                #'INx', 'INy', 'INz',
                #'Px', 'Py', 'Pz',
                #'Ix', 'Iy', 'Iz',
                'Dx', 'Dy', 'Dz',
                'AAx', 'AAy', 'AAz',
                #'PIfx', 'PIfy', 'PIfz',
                'AAfx', 'AAfy', 'AAfz',
                #'SLx', 'SLy', 'SLz',
            ),
            #region=(40,50),
    )	

    # analyzeCSV(
    #            gamefile('Tardegrade.AttitudeControl.csv'),
    #            (
    #                'Alt',
    #                'Ex', 'Ey', 'Ez',
    #                'Sx', 'Sy', 'Sz',
    #                'AVx', 'AVy', 'AVz',
    #                'INx', 'INy', 'INz',
    #                'Ax', 'Ay', 'Az',
    #                'Px', 'Py', 'Pz',
    #                'Ix', 'Iy', 'Iz',
    #                'Dx', 'Dy', 'Dz',
    #                'AAx', 'AAy', 'AAz',
    #                'PIfx', 'PIfy', 'PIfz',
    #                'AAfx', 'AAfy', 'AAfz',
    #                'SLx', 'SLy', 'SLz',
    #                'ODx', 'ODy', 'ODz',
    #                'ODmx', 'ODmy', 'ODmz',
    #            ),
    #            (
    #                'Alt', 'Ex', 'Sx', 'AVx', 'Ax', #'INx',
    #                'AAx', 'AAfx', 'PIfx', 'SLx',
    #                'Px', 'Dx',
    #                # 'ODx', 'ODmx',
    #            ),
    #            region=(15,2000),
    #            #  region=(0,490),
    #            )

#     analyzeCSV('VS-filtering-43.csv',
#                ('AbsAlt', 'TerAlt', 'Alt', 'AltAhead', 'Err', 'VSP', 'VSF', 'MinVSF', 'aV', 'rV', 'dV', 'mdTWR', 'mTWR', 'hV'),
#                 ('AbsAlt', 'TerAlt', 'Alt', 'AltAhead', 'Err', 'VSP', 'VSF', 'aV', 'rV', 'dV', 'mdTWR', 'hV'))
#                 ('Alt', 'Err', 'VSP', 'VSF', 'dV'))
# #                ['AltAhead']
#                 , (13,))

#    analyzeCSV('Debugging/MAN1.csv',
#               ('name',
#                'UT', 'dV', 'angMod', 'nextThrottle', 'Throttle', 'TTB'),
#               ('dV', 'angMod', 'nextThrottle', 'Throttle', 'TTB'),
#               x='UT')

#     analyzeCSV('Debugging/vertical-overshooting-bug.csv',
#                ('name',
#                 'Altitude', 'TerrainAltitude', 'RelVerticalSpeed', 'VerticalSpeed',
#                 'VerticalCutoff', 'setpoint', 'setpoint_correction', 'VerticalAccel',
#                 'upAF', 'K', 'VSP'),
#                ('TerrainAltitude', 'RelVerticalSpeed', 'VerticalSpeed',
#                 'VerticalCutoff', 'setpoint', 'setpoint_correction', 'VerticalAccel',
#                 'K', 'VSP'))

# analyzeCSV('Tests/REN.csv',
#           ('TimeToStart', 'TimeToTarget', 'DeltaTA', 'DeltaFi', 'DeltaR', 'DistanceToTarget', 'dVr', 'dVn',
#            'dVp'),
#           region=[0])

# analyzeCSV('Tests/BJ.csv',
#           ('last dist', 'dist', 'delta'),
#           region=[0])

#     analyzeCSV('VS-filtering-39.csv',
#                ('BestAlt', 'DetAlt', 'AltAhead')
#                )

# analyzeCSV('Tests/ATC.csv',
#           ('error',
#            'error.x', 'error.y', 'error.z',
#            'steer.x', 'steer.y', 'steer.z',
#            'pid.x', 'pid.y', 'pid.z',
#            'vel.x', 'vel.y', 'vel.z'),
#            region=[0])

# analyzeCSV(
#         'Tests/DEO-Kerbin-hard.csv',
#         # 'Tests/DEO-Duna-hard.csv',
#            (
#                'Alt',
#                'P',
#                'Rho',
#                'Rho/ASL',
#                'T',
#                'G',
#                'Speed',
#                'HSpeed',
#                'VSpeed',
#                'Mach',
#                'dP',
#                'Err'
#            ),
#            (
#                'Alt',
#                'P',
#                'Rho',
#                'Rho/ASL',
#                'T',
#                'G',
#                'Speed',
#                'HSpeed',
#                'VSpeed',
#                'Mach',
#                'dP',
#                'Err'
#            ),
#            )
