from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.mlab import PCA
from scipy.spatial import ConvexHull
from matplotlib.path import Path

from itertools import combinations

def plot_search_paths(N, filename = 'CDOS_test.csv', num_paths = 3):
    os.chdir('/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/KSP_test_1.2.2/')
    df = pd.read_csv(filename,
                     names=['tag', 'startT', 'transfer', 'dist', 'dir_x', 'dir_y', 'dDist', 'feasible', 'time'])
    # get samples
    starts = df[df['tag'] == 'initial point'].index.tolist()
    samples = []
    prev = starts[0]
    for i in starts[1:]:
        samples.append(df.loc[prev:i - 1, :])
        prev = i
    samples.append(df.loc[prev:, :])
    # plot each in turn
    # Choose a color map, loop through the colors, and assign them to the color
    # cycle. You need NPOINTS-1 colors, because you'll plot that many lines
    # between pairs. In other words, your line is not cyclic, so there's
    # no line from end to beginning
    cm = plt.get_cmap('gist_rainbow')
    fig = plt.figure()
    ax = fig.add_subplot(211)
    n = (N - 1) * 3
    lines = ['o-', '*-', '^-']*(num_paths/3+1)
    for l, s in enumerate(samples[n:n + num_paths]):
        print(s.to_string()+'\n')
        path = s.loc[:, 'startT':'dist']
        maxT = max(max(path.startT), max(path.transfer)) + 1
        minT = min(min(path.startT), min(path.transfer)) - 1
        points = len(path.startT)
        ax.set_prop_cycle('color', [cm(1. * i / (points - 1)) for i in range(points - 1)])
        for i in range(points - 1):
            ax.plot(path.startT[i:i + 2], path.transfer[i:i + 2], lines[l])
            # plt.plot(path.startT, path.transfer, 'x-')
            # plt.axis([minT, maxT, minT, maxT])
    s1 = samples[n]
    s1 = s1.loc[s1.tag == 'scan start']
    s2 = samples[n+1]
    s2 = s2.loc[s2.tag == 'scan start']
    ax2 = fig.add_subplot(212)
    ax2.plot(s1.startT, list(s1.dist), 'b.-')
    ax2.plot(s2.startT, list(s2.dist), 'r.-')
    plt.show()


def center_angle(a):
    return abs(abs(a)-90)


def plot_dV_PCA():
    os.chdir('/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/KSP_test_1.2.2')
    names = ['cdos_dV', 'cdos_d', 'cdos_time',
             'correction', 'correction_time',
             'cdos_ttr', 'cdos_ttr_end', 'cdos_ttr_d', 'cdos_ttr_time',
             'old', 'old_end', 'old_d', 'old_time',
             'correction_dV_diff', 'old_dV_diff',
             'incl',
             'PeR', 'ApR',
             'PeA_angle_before', 'PeA_angle_after',
             'ttr_before', 'ttr_after',
             'res_before', 'res_after',
             'signed_ttr_after',
             'periodT', 'periodV_before', 'periodV_after',
             'eccV_before', 'eccV_after', 'eccT',
             'enV_before', 'enV_after', 'enT',
             'ttr_up',  'ttr_async',
             'time']
    df = pd.read_csv('CDOS_dV.csv', names=names)
    # get samples
    # df = df.iloc[:35,:]
    df = df.loc[df.correction > 0.1]
    classes = df.correction_dV_diff < -1
    # classes = (df.cdos_ttr - df.cdos_dV) < -1

    # transform some dimensions
    # df.cdos_dV = df.cdos_dV / df.correction
    # df.correction = df.correction /df.cdos_dV
    df.res_before = abs(df.res_before)
    df.res_after = abs(df.res_after)

    df.PeA_angle_before = center_angle(df.PeA_angle_before)
    df.PeA_angle_after = center_angle(df.PeA_angle_after)

    df.PeA_angle_after -= df.PeA_angle_before
    df.ttr_after -= df.ttr_before
    df.res_after -= df.res_before

    df.periodV_before /= df.periodT
    df.periodV_after /= df.periodT

    df.eccV_before -= df.eccT
    df.eccV_after -= df.eccT
    df.enV_before /= df.enT
    df.enV_after /= df.enT

    # df.enV_after = (df.enV_after-df.enV_before)

    # use only these dimensions
    dims = [
        # 'correction_dV_diff',
        #     'cdos_dV',
            'correction',
            'correction_time',
            'incl',
            # 'PeR',
            # 'ApR',
            'PeA_angle_before',
            'PeA_angle_after',
            # 'ttr_before',
            'ttr_after',
            # 'res_before',
            'res_after',
            'signed_ttr_after',
            # 'periodT',
            'periodV_before',
            'periodV_after',
            'eccV_before',
            'eccV_after',
            # 'eccT',
            'enV_before',
            'enV_after',
            # 'enT',
            # 'ttr_up',
            # 'ttr_async'
            ]

    def pca_var(sub_dims):
        data = np.array([df[d] for d in sub_dims]).T
        try: pca = PCA(data, standardize=True)
        except: return 0,1,0,1,None,None,None,sub_dims

        classed_points = zip(classes, pca.Y)
        pos = [(it[0], it[1]) for c, it in classed_points if c]
        neg = [(it[0], it[1]) for c, it in classed_points if not c]
        P_hull = [pos[i] for i in ConvexHull(pos).vertices]; P_hull.append(P_hull[0])
        N_hull = [neg[i] for i in ConvexHull(neg).vertices]; N_hull.append(N_hull[0])
        P_hull = np.array(P_hull)
        N_hull = np.array(N_hull)
        P_path = Path(P_hull)
        N_path = Path(N_hull)

        N_sep = 0
        for it in neg:
            if not P_path.contains_point(it):
                N_sep += 1

        P_sep = 0
        for it in pos:
            if not N_path.contains_point(it):
                P_sep += 1

        return N_sep, float(len(neg)), P_sep, float(len(pos)), P_hull, N_hull, pca, sub_dims

    def plot_pca(N_sep, N_total, P_sep, P_total, P_hull, N_hull, pca, sub_dims):
        print ('\nPC variation:\n' + '\n'.join(str(f) for f in pca.fracs) + '\n')
        print (', '.join(sub_dims))
        for wt in pca.Wt:
            print (', '.join('%+.2f' % w for w in wt))

        print('\ncenter:\n%s' % '\n'.join('%s: %f' % (sub_dims[i], pca.mu[i]) for i in range(len(sub_dims))))
        print('\nstd:\n%s' % '\n'.join('%s: %f' % (sub_dims[i], pca.sigma[i]) for i in range(len(sub_dims))))

        # print ('\nPositive bounds: [%f : %f]:' % tuple(T_bounds))
        # minB = (pca.Wt[0] * T_bounds[0]+ pca.mu).T
        # maxB = (pca.Wt[0] * T_bounds[1]+ pca.mu).T
        # for i, (bmin, bmax) in enumerate(zip(minB, maxB)):
        #     print ('%s: [%f : %f]' % (sub_dims[i], bmin, bmax))
        print('\nP separation: %d/%d = %f' % (P_sep, P_total, P_sep / float(P_total)))
        print('\nN separation: %d/%d = %f' % (N_sep, N_total, N_sep / float(N_total)))

        pc = np.array([(it[0], it[1], it[2]) for it in pca.Y]).T
        col = ['#0000FF' if s else '#FF0000' for s in classes]

        plt.scatter(pc[0], pc[1], color=col)
        plt.plot(P_hull.T[0], P_hull.T[1], 'b--')
        plt.plot(N_hull.T[0], N_hull.T[1], 'r--')
        plt.show()

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(pc[0], pc[1], pc[2], c=col)
        # plt.show()

    max_sep = sorted((pca_var(c) for c in combinations(dims, 4)),
                     key=lambda x: x[0]/x[1]+x[2]/x[3],
                     # key=lambda x: x[0] + x[2],
                     # key=lambda x: x[0],
                     reverse=True)[:20]

    [plot_pca(*s) for s in max_sep]

    # plot_pca(*pca_var([
    #     # 'cdos_dV',
    #     # 'correction',
    #     'incl',
    #     # 'PeR',
    #     # 'ApR',
    #     # 'PeA_angle_before',
    #     'PeA_angle_after',
    #     # 'ttr_before',
    #     # 'ttr_after',
    #     # 'res_before',
    #     # 'res_after',
    #     # 'periodT',
    #     # 'periodV_before',
    #     # 'periodV_after',
    #     'eccV_after',
    #     # 'enV_after',
    #     # 'ttr_up',
    #     # 'ttr_async'
    # ]))


if __name__ == '__main__':
    plot_dV_PCA()
    # plot_search_paths(5, num_paths=3)

#best so far
# incl, PeA_angle_after, eccV_after
# +0.64, -0.56, -0.52
# -0.07, +0.63, -0.77
# -0.76, -0.53, -0.36
#
# divider: PC1 = 0.5
#
# center:
# incl: 8.474319
# PeA_angle_after: 20.285899
# eccV_after: -0.076491
#
# std:
# incl: 6.042648
# PeA_angle_after: 18.780472
# eccV_after: 0.074847
