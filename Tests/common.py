import sys, os
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

dt = 0.02

gamedir = u'/home/storage/Games/KSP_linux/PluginsArchives/Development/AT_KSP_Plugins/KSP-test/'
game = u'KSP_test_1.3'


def gamefile(filename): return os.path.join(gamedir, game, filename)


def clampL(x, L): return x if x > L else L


def clampH(x, H): return x if x < H else H


def clamp(x, L, H):
    if x > H: return H
    elif x < L: return L
    return x


def clamp01(x):
    if x > 1: return 1.0
    elif x < 0: return 0.0
    return x


vclamp01 = np.vectorize(clamp01)


def lerp(f, t, time): return f + (t - f) * clamp01(time)


def center_deg(a):
    a %= 360
    if a > 180: a -= 360
    return a


def asymp01(x, k=1):
    return 1.0 - 1.0 / (x / k + 1.0)


def fit_plot(xydata, model):
    data = np.array(xydata, float)
    xdata, ydata = data[:, 0], data[:, 1]
    opt, cov = curve_fit(model, xdata, ydata)
    yfit = model(xdata, *opt)
    plt.plot(xdata, ydata, '.')
    plt.plot(xdata, yfit, '-x')
    return opt


def plt_show_maxed():
    plt.tight_layout(pad=0, h_pad=0, w_pad=0)
    plt.subplots_adjust(top=0.99, bottom=0.03, left=0.05, right=0.99, hspace=0.25, wspace=0.1)
    mng = plt.get_current_fig_manager()
    try:
        mng.window.showMaximized()
    except AttributeError:
        try:
            mng.frame.Maximize(True)
        except AttributeError:
            try:
                mng.full_screen_toggle()
            except AttributeError:
                pass
    plt.show()


def legend():
    plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0.)


def color_grad(num, repeat=1):
    colors = [(0, 1, 0)] * repeat
    n = float(num)
    for i in range(1, num):
        c = (i / n, 1 - i / n, 1 - i / n)
        colors += (c,) * repeat
    return colors


class vec(object):
    def __init__(self, x=0, y=0, z=0):
        self.v = np.array([float(x), float(y), float(z)])

    @property
    def xzy(self):
        return vec(self[0], self[2], self[1])

    @property
    def norm(self):
        return vec.from_array(np.array(self.v) / np.linalg.norm(self.v))

    def __str__(self):
        return '[%+.4f, %+.4f, %+.4f] |%.4f|' % tuple(list(self.v) + [abs(self), ])

    def __repr__(self): return str(self)

    def __getitem__(self, index): return self.v[index]

    def __setitem__(self, index, value): self.v[index] = value

    def __nonzero__(self): return bool(np.any(self.v))

    def __abs__(self):
        return np.linalg.norm(self.v)

    def __add__(self, other):
        return vec.from_array(self.v + other.v)

    def __iadd__(self, other):
        self[0] += other[0]
        self[1] += other[1]
        self[2] += other[2]
        return self

    def __sub__(self, other):
        return vec.from_array(self.v - other.v)

    def __neg__(self):
        return self * -1

    def __rmul__(self, other): return self * other

    def __mul__(self, other):
        if isinstance(other, vec):
            return np.dot(self.v, other.v)
        elif isinstance(other, (int, float)):
            return vec.from_array(self.v * float(other))
        raise TypeError('vec: unsupported multiplier type %s' % type(other))

    def __div__(self, other):
        return self * (1.0 / other)

    def cross(self, other):
        return vec.from_array(np.cross(self.v, other.v))

    def project(self, onto):
        m2 = onto * onto
        return onto * (self * onto / m2)

    def angle(self, other):
        """
		:rtype float: radians
		"""
        return np.arccos(self * other / (abs(self) * abs(other)))

    def cube_norm(self):
        return self / max(abs(x) for x in self)

    @classmethod
    def from_array(cls, a):
        assert len(a) == 3, 'Array should be 1D with 3 elements'
        return vec(a[0], a[1], a[2])

    @classmethod
    def sum(cls, vecs): return sum(vecs, vec())

    @classmethod
    def rnd(cls, magnitude=1):
        return cls.from_array(np.random.rand(3) * 2 - 1).norm * magnitude


class vec6(object):
    def __init__(self):
        self.positive = vec()
        self.negative = vec()

    def add(self, v):
        for i, d in enumerate(v):
            if d >= 0: self.positive[i] += d
            else: self.negative[i] += d

    def sum(self, vecs):
        for v in vecs: self.add(v)

    def clamp(self, v):
        c = vec()
        for i, d in enumerate(v):
            if d >= 0: c[i] = min(d, self.positive[i])
            else: c[i] = max(d, self.negative[i])
        return c

    def __str__(self):
        return 'positive: %s\nnegative: %s' % (self.positive, self.negative)

    def __repr__(self): return str(self)


def xzy(v): return [v[0], v[2], v[1]]


class PID(object):
    def __init__(self, p, i, d, min_a, max_a):
        self.P = float(p)
        self.I = float(i)
        self.D = float(d)
        self.min = float(min_a)
        self.max = float(max_a)
        self.reset()

    def setPID(self, p, i, d):
        self.P = p
        self.I = i
        self.D = d
        self.reset()

    def setFrom(self, pid):
        self.P = pid.P
        self.I = pid.I
        self.D = pid.D
        self.reset()

    def pack(self):
        return [self.P, self.I, self.D]

    def reset(self):
        self.value = 0
        self.ierror = 0
        self.perror = 0
        self.action = 0

    def update(self, err):
        if self.perror == 0:
            self.perror = err
        return self.update2(err, (err - self.perror) / dt)

    def update2(self, err, spd):
        if self.ierror * err < 0:
            self.ierror = 0
        old_ierror = self.ierror
        self.ierror += err * dt
        act = self.P * err + self.I * self.ierror + self.D * spd
        clamped = clamp(act, self.min, self.max)
        if clamped != act:
            self.ierror = old_ierror
        self.perror = err
        self.action = clamped
        return self.action

    def sim(self, PV, SV, T):
        V = [0.0]
        for sv in SV[1:]:
            act = self.update(sv - V[-1])
            if np.isinf(act) or np.isnan(act):
                act = sys.float_info.max
            V.append(V[-1] + act)
        # plot
        V = np.array(V);
        SV = np.array(SV)
        plt.subplot(3, 1, 1)
        plt.plot(T, SV)
        plt.subplot(3, 1, 2)
        plt.plot(T, V)
        plt.subplot(3, 1, 3)
        plt.plot(T, V - SV)

    def __str__(self, *args, **kwargs):
        return ('p % 8.5f, i % 8.5f, d % 8.5f, min % 8.5f, max % 8.5f, ierror % 8.5f; action = % 8.5f'
                % (self.P, self.I, self.D, self.min, self.max, self.ierror, self.action))


class PID2(PID):
    def update(self, err):
        if self.perror == 0:
            self.perror = err
        return self.update2(err, (err - self.perror) / dt)

    def update2(self, err, spd):
        if self.ierror * err < 0:
            self.ierror = 0
        d = self.D * spd
        self.ierror = clamp(self.ierror + self.I * err * dt if abs(d) < 0.6 * self.max else 0.9 * self.ierror,
                            self.min, self.max)
        self.perror = err
        self.action = clamp(self.P * err + self.ierror + d, self.min, self.max)
        return self.action


class PID3(PID):
    def __init__(self, p,i,d, min_a, max_a, filter_tau):
        super(PID3, self).__init__(p,i,d, min_a, max_a)
        self.filter = Filter(1)
        self.filter.setTau(filter_tau)

    def update(self, err):
        if self.perror == 0:
            self.perror = err
        return self.update2(err, (err - self.perror) / dt)

    def update2(self, err, spd):
        if self.ierror * err < 0:
            self.ierror = 0
        old_ierror = self.ierror
        self.ierror += err * dt
        d = self.D * self.filter.EWA(spd)
        act = self.P * err + self.I * self.ierror + d
        clamped = clamp(act, self.min, self.max)
        if clamped != act:
            self.ierror = old_ierror
        self.action = clamped
        self.perror = err
        return self.action


class Filter(object):
    def __init__(self, ratio):
        self.ratio = ratio
        self.cur = 0

    def setTau(self, tau):
        self.ratio = dt/(tau+dt)

    def Gauss(self, new, poles=2):
        for _i in range(poles):
            self.cur = self.cur + (new - self.cur) * self.ratio
        return self.cur

    def EWA(self, new):
        self.cur = self.cur+(new-self.cur) * self.ratio
        return self.cur

    def EWA2(self, new):
        if new < self.cur:
            ratio = clamp01(1 - self.ratio)
        else:
            ratio = self.ratio
        self.cur = self.cur + (new - self.cur) * ratio
        return self.cur

    def Equilibrium(self, new):
        if new * self.cur < 0:
            return self.EWA(new)
        else:
            self.cur = new
            return self.cur


def vFilter(v, flt, **kwargs):
    f = [v[0]]
    for val in v[:-1]:
        f.append(flt(f[-1], val, **kwargs))
    return np.fromiter(f, float)


class SimpleKalman(object):
    def __init__(self, Q, R):
        self.Q = Q
        self.R = R
        self._X = 0
        self._P = 1
        self._K = 1

    @property
    def value(self):
        return self._X

    def update(self, measurement):
        # time update
        P = self._P + self.Q
        X = self._X

        # measurement update
        self._K = P / (P + self.R)
        self._X = X + self._K * (measurement - X)
        self._P = (1 - self._K) * self._P
        return self._X
