'''
Created on Jan 8, 2015

@author: Allis Tauri
'''

import numpy as np
import matplotlib.pyplot as plt
import sys


def clampL(x, L): return x if x > L else L

def clamp(x, L, H): 
    if x > H: return H
    elif x < L: return L
    return x
#end def

def clamp01(x): 
    if x > 1: return 1.0
    elif x < 0: return 0.0
    return x
#end def

def asymp01(x, k=1):
    return 1.0-1.0/(x/k+1.0); 

vclamp01 = np.vectorize(clamp01)

class vec(object):
    def __init__(self, x=0, y=0, z=0):
        self.v = np.array([float(x), float(y), float(z)])

    @property
    def norm(self): 
        return vec.from_array(np.array(self.v)/np.linalg.norm(self.v))
    
    def __str__(self):
        return '[%+.4f, %+.4f, %+.4f] |%.4f|' % tuple(list(self.v)+[abs(self),])
    
    def __repr__(self): return str(self)
    
    def __getitem__(self, index): return self.v[index]
    def __setitem__(self, index, value): self.v[index] = value
    
    def __abs__(self):
        return np.linalg.norm(self.v)
    
    def __add__(self, other):
        return vec.from_array(self.v+other.v)
    
    def __iadd__(self, other):
        self[0] += other[0]
        self[1] += other[1]
        self[2] += other[2]
        return self
    
    def __sub__(self, other):
        return vec.from_array(self.v-other.v)
    
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
        return self * (1.0/other)
    
    def cross(self, other):
        return vec.from_array(np.cross(self.v, other.v))

    def project(self, onto):
        m2 = onto*onto
        return onto * (self*onto/m2)
    
    def angle(self, other):
        return np.rad2deg(np.arccos(self*other/(abs(self)*abs(other))))
    
    def cube_norm(self):
        return self/max(abs(x) for x in self)
    
    @classmethod
    def from_array(cls, a):
        assert len(a) == 3, 'Array should be 1D with 3 elements'
        return vec(a[0], a[1], a[2])
    
    @classmethod
    def sum(cls, vecs): return sum(vecs, vec())
#end class

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
#end class

def lerp(f, t, time): return f+(t-f)*clamp01(time)

class engine(object):
    def __init__(self, spec_torque, min_thrust=0.0, max_thrust=100.0, maneuver=False, manual=False):
        self.torque = spec_torque
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust 
        self.limit  = 1.0
        self.limit_tmp = 1.0
        self.best_limit = 1.0
        self.current_torque = vec()
        self.maneuver = maneuver
        self.manual = manual
        
    def nominal_current_torque(self, K):
            return (self.torque *
                    (lerp(self.min_thrust, self.max_thrust, K) if not self.manual
                     else self.max_thrust))
            
    def vsf(self, K): return (1 if self.maneuver else K)
#end class

class PID(object):
    dt = 0.01
    
    def __init__(self, kp, ki, kd, min_a, max_a):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min = float(min_a)
        self.max = float(max_a)
        
        self.value  = 0
        self.ierror = 0
        self.perror = 0
    #end def
        
    def update(self, err):
        old_ierror = self.ierror
        self.ierror += err*self.dt
        act = self.kp*err + self.ki*self.ierror + self.kd*(err-self.perror)/self.dt
        clamped = clamp(act, self.min, self.max)
        if clamped != act: self.ierror = old_ierror
        self.perror = err
        return clamped
    #end def
    
    def sim(self, PV, SV, T):
        V = [0.0]
        for sv in SV[1:]:
            act = self.update(sv-V[-1])
            if np.isinf(act) or np.isnan(act): 
                act = sys.float_info.max
            V.append(V[-1]+act)
        #plot
        V = np.array(V); SV = np.array(SV)
        plt.subplot(3,1,1)
        plt.plot(T, SV)
        plt.subplot(3,1,2)
        plt.plot(T, V)
        plt.subplot(3,1,3)
        plt.plot(T, V-SV)
#end class

class PID2(PID):
    def update(self, err):
        d = self.kd * (err-self.perror)/self.dt
        self.ierror = self.ierror + self.ki * err * self.dt if abs(d) < 0.6*self.max else 0.9 * self.ierror
        return clamp(self.kp*err + self.ierror + d, self.min, self.max)

class VSF_sim(object):
    dt     = 0.01
    t1     = 5.0
    
    def __init__(self, K1, L1, K2, L2):
        self.K1 = K1
        self.L1 = L1
        self.K2 = K2
        self.L2 = L2
        
        self.maxV = 0.0
        self.VSP  = 0.0
        self.upA  = 0.0
        self.upV  = 0.0
        self.upX  = 0.0
        self.E    = 0.0
        
        self.T = []
        self.X = []
        self.V = []
        self.K = []
        self.F = []
        
        self.M = 4.0
    #end def
        
    def vK1(self, E):
#         if self.upV <= self.maxV:
            return clamp01(E/2.0/(clampL(self.upA/self.K1+1.0, self.L1))**2)
#         else: 
#             return clamp01(E*self.upA/(clampL(-self.K2*E, self.L2))**2)
        
    def vK2(self, E): 
#         if self.maxV > self.upV:
#         self.VSP = self.maxV+(self.maxV-self.upV)*(1-self.dt/0.04)
        
        self.VSP = self.maxV+2.0*(1/self.twr)**1
#         else: self.VSP = self.maxV
#         print self.VSP
        return self.vK1(E)
            
#         return self.pid.update(E)
    
    _vK = vK1
        
    @property
    def vK(self): return self._vK(self.E)
    
    def run(self, upV, maxV=1.0, thrust=20.0, vK = None):
        self.thrust = thrust
        self.twr = thrust/9.81/self.M
        self.pid = PID(0.1, 0.01, 0.051, 0, 1)
        if vK is not None: self._vK = vK
        
        t  = 0
        self.maxV = maxV
        self.VSP  = maxV
        self.upA  = 0.0
        self.upV  = upV
        self.upX  = 100.0
        
        self.T = [t]
        self.V = [self.upV]
        self.X = [self.upX]
        self.K = [self.vK]
        self.F = [0.0]
        
        while t < self.t1:
            self.upA += (self.thrust*self.K[-1] - self.M*9.81)/self.M
            self.upV += self.upA*self.dt
            self.upX += self.upV*self.dt
            self.E    = self.VSP-self.upV
            t += self.dt
            self.T.append(t)
            self.F.append(self.thrust*self.K[-1] * self.dt)
            self.V.append(self.upV)
            self.X.append(self.upX)
            self.K.append(self.vK)
        
#         print(("Start velocity: %f\n"
#                "K1 %f; L1 %f; K2 %f L2 %f\n"
#                "Fuel consumed: %f\n") 
#               % (upV, self.K1, self.L1, self.K2, self.L2, sum(self.F)))
    #end def
    
    def plot(self, r, c, n):
        plt.subplot(r, c, n)
        plt.plot(self.T, self.V, label=("V: twr=%.1f" % self.twr))
        plt.xlabel("time (s)")
        plt.ylabel("vertical speed (m/s)")
        plt.legend()
        
#         plt.subplot(2, 1, 2)
#         plt.plot(self.T, self.K, label=("vK: twr=%.1f" % self.twr))
#         plt.xlabel("time (s)")
#         plt.ylabel("thrust coefficient")
#         plt.legend()
    #end def
#end class

def sim_PID():
    np.random.seed(42)
    
    def rSV(val, delta): 
        return max(0, min(100, 
                          val + (delta * np.random.random()-delta/2.0)))

    t1  = 10.0
    PV  = 10.0
    SV  = [rSV(50, 100)]
    T   = np.arange(0, t1, PID.dt)
    sSV = SV[-1]
    for _t in T[1:]:
        r = np.random.random()
        if r < 0.98:
            SV.append(SV[-1])
            continue
        sSV = rSV(sSV, 50)
        SV.append(sSV)
    
    pid1 = PID(0.8, 0.2, 0.001, -10, 10)
    pid2 = PID2(0.8, 0.2, 0.001, -10, 10)
    
    pid1.sim(PV, SV, T)
    pid2.sim(PV, SV, T)
    plt.show()
    

def sim_PIDf():
    MoI = vec(3.517439, 5.191057, 3.517369)
    def A(_t):  return vec(_t/MoI[0], _t/MoI[1], _t/MoI[2])
    def Tf(_t): return vec(MoI[0]/_t, MoI[1]/_t, MoI[2]/_t)
    ph = 0.4; pl = 0.05; dp = ph-pl 
    
    def hill(x, x0=1.0, k=1.0):
        p1 = x**2
        p2 = -(x*2-x0*3)
        p3 = (x*2+x0*3)
        return p1*p2*p3
    
    T = np.arange(0, 2, 0.01)
    P = []; I = []; 
    As = []; S = []; S1 = []; S2 = [];
    for t in T:
        As.append(abs(A(t)))
#         f = asymp01(As[-1], 10)
#         P.append(pl+dp*(1.0-f))
#         I.append(P[-1]*(1.0-f)/1.5)
        S.append(hill(t, 1, 1))
        
#     plt.plot(T, P)
#     plt.plot(T, I)
#     plt.show()
    plt.plot(T, S)
    plt.show()


# def tI_PCA():
#     from matplotlib.mlab import PCA
#     ships = 
#     [
#      
#      ]

Quadro_Manual = [
                    engine(vec(-0.8, 0.0, 0.0),  min_thrust=0.0, max_thrust=18.0, manual=True),
                    engine(vec(1.8, 0.0, 1.8),   min_thrust=0.0, max_thrust=40.0),# maneuver=True),
                    engine(vec(1.8, 0.0, -1.8),  min_thrust=0.0, max_thrust=40.0),# maneuver=True),
                    engine(vec(-1.8, 0.0, -1.8), min_thrust=0.0, max_thrust=40.0),# maneuver=True),
                    engine(vec(-1.8, 0.0, 1.8),  min_thrust=0.0, max_thrust=40.0),# maneuver=True),
                 ]
    
VTOL_Test = [
                engine(vec(-3.4, -2.0, 0.0), min_thrust=0.0, max_thrust=250.0),
                engine(vec(-3.4, 2.0, 0.0),  min_thrust=0.0, max_thrust=250.0),
                engine(vec(3.5, -2.0, 0.0),  min_thrust=0.0, max_thrust=250.0),
                engine(vec(1.5, 2.0, 0.0),   min_thrust=0.0, max_thrust=250.0),
                engine(vec(1.3, 2.0, 1.6),   min_thrust=0.0, max_thrust=20.0, maneuver=True),
                engine(vec(3.1, -2.0, -3.7), min_thrust=0.0, max_thrust=20.0, maneuver=True),
                engine(vec(-3.1, -2.0, 3.7), min_thrust=0.0, max_thrust=20.0, maneuver=True),
                engine(vec(-2.4, 2.0, -2.9), min_thrust=0.0, max_thrust=20.0, maneuver=True),
             ]

Hover_Test = [
                engine(vec(-6.2, 6.4, 0.6),   max_thrust=450),
                engine(vec(-6.2, -6.4, -0.6), max_thrust=450),
                engine(vec(3.9, 7.4, 0.6),    max_thrust=450),
                engine(vec(3.9, -6.4, -0.6),  max_thrust=450)
            ]

VTOL_Test_Bad_Demand = [
                        vec(0.0, 0.0, 0.0),
                        vec(10.0, 0.0, 0.0),
                        vec(0.0, 10.0, 0.0),
                        vec(0.0, 0.0, 10.0),
                            
                        vec(-1797.147, 112.3649, 80.1167), #Torque Error: 165.3752
                        vec(1327.126, -59.91731, 149.1847), #Torque Error: 229.8387
                        vec(107.5895, -529.4326, -131.0672), #Torque Error: 59.95838
                        vec(50.84914, -1.706408, 113.4622), #Torque Error: 25.10385
                        vec(-0.4953138, 0.2008617, 39.52808),
                        vec(14.88248, 0.9660782, -51.20171),
                        vec(-20.34281, -10.67025, 38.88113),
                        ]

Hover_Test_Bad_Demand = [
                         vec(0.2597602, -5.2778279, 0.8444),
                         vec(0.9042788, -1.347212, 483.4898), #Torque Error: 483.4926kNm, 90deg
                         vec(-0.7012861, 0.2607862, -294.2217), #Torque Error: 339.2753kNm, 95.00191deg
                         vec(0.2597602, -0.2778279, 295.8444), #Torque Error: 341.1038kNm, 94.96284deg
                         ]
    
def sim_Attitude():
    def S(T, K, L=0): return vec.sum(Tk(T, K, L))
    def Sm(T, K, L=0): return abs(S(T,K))
    def Tk(T, K, L=0): return [t*clampL(k, L) for t, k in zip(T,K)]
    
    def opt(target, engines, vK, eps):
        tm = abs(target)
        comp = vec()
        man  = vec()
        for e in engines:
            if not e.manual: 
                e.limit_tmp = -e.current_torque*target/tm/abs(e.current_torque)
                if e.limit_tmp > 0:
                    comp += e.nominal_current_torque(e.vsf(vK)*e.limit)
                elif e.maneuver:
                    if e.limit == 0: e.limit = eps
                    man += e.nominal_current_torque(e.vsf(vK)*e.limit)
                else: e.limit_tmp = 0
            else: e.limit_tmp = 0
        compm = abs(comp)
        manm = abs(man)
        if compm < eps and manm == 0: return False
        limits_norm = clamp01(tm/compm)
        man_norm = clamp01(tm/manm)
        for e in engines:
            if e.manual: continue
            if e.limit_tmp < 0:
                e.limit = clamp01(e.limit * (1.0 - e.limit_tmp*man_norm))
            else:
                e.limit = clamp01(e.limit * (1.0 - e.limit_tmp*limits_norm))
        return True
    
    def optR(engines, D=vec(), vK=1.0, eps=0.1, maxI=500, output=True):
        torque_clamp = vec6()
        torque_imbalance = vec()
        ti_min = vec()
        for e in engines:
            e.limit = 1.0 if not e.maneuver else 0
            ti_min += e.nominal_current_torque(0)
        if abs(ti_min) > 0:
            print ti_min
            anti_ti_min = vec()
            for e in engines:
                if e.torque*ti_min < 0:
                    anti_ti_min += e.nominal_current_torque(1)
            if abs(anti_ti_min) > 0:
                vK = clampL(vK, clamp01(abs(ti_min)/abs(anti_ti_min)*1.2))
        for e in engines:
            e.current_torque = e.nominal_current_torque(e.vsf(vK))
            torque_imbalance += e.nominal_current_torque(e.vsf(vK) * e.limit)
            torque_clamp.add(e.current_torque)
        _d = torque_clamp.clamp(D)
        if output:
            print 'vK: %f' % vK
            print 'Torque clamp:\n', torque_clamp
            print 'demand:        ', D
            print 'clamped demand:', _d
            print ('initial     %s, error %s, dir error %s' % 
                   (torque_imbalance, abs(torque_imbalance-_d), torque_imbalance.angle(_d)))
        s = []; s1 = []; i = 0;
        best_error = -1; best_angle = -1; best_index = -1;
        for i in xrange(maxI):
            s.append(abs(torque_imbalance-_d))
            s1.append(torque_imbalance.angle(_d) if abs(_d) > 0 else 0)
            if(s1[-1] <= 0 and s[-1] < best_error or 
               s[-1]+s1[-1] < best_error+best_angle or best_angle < 0):
                for e in engines: e.best_limit = e.limit
                best_error = s[-1]
                best_angle = s1[-1]
                best_index = len(s)-1
#             if len(s1) > 1 and s1[-1] < 55 and s1[-1]-s1[-2] > eps: break
#             if s1[-1] > 0:
#                 if s1[-1] < eps or len(s1) > 1 and abs(s1[-1]-s1[-2]) < eps*eps: break
#             elif 
            if s[-1] < eps or len(s) > 1 and abs(s[-1]-s[-2]) < eps/10.0: break
            if len(filter(lambda e: e.manual, engines)) == 0:
                mlim = max(e.limit for e in engines)
                if mlim > 0: 
                    for e in engines: e.limit = clamp01(e.limit/mlim)
            if not opt(_d-torque_imbalance, engines, vK, eps): break
            torque_imbalance = vec.sum(e.nominal_current_torque(e.vsf(vK) * e.limit) for e in engines)
        for e in engines: e.limit = e.best_limit
        if output:
            ##########
            print 'iterations:', i+1
#             print 'dAngle:    ', abs(s1[-1]-s1[-2])
            print 'limits:    ', list(e.limit for e in engines)
            print 'result      %s, error %s, dir error %s' % (torque_imbalance, s[best_index], s1[best_index])
            print 'engines:\n'+'\n'.join(str(e.nominal_current_torque(e.vsf(vK) * e.limit)) for e in engines)
            print
            ##########
            x = np.arange(len(s))
            plt.subplot(2,1,1)
            plt.plot(x, s, '-')
            plt.xlabel('iterations')
            plt.ylabel('torque error (kNm)')
            plt.subplot(2,1,2)
            plt.plot(x, s1, '-')
            plt.xlabel('iterations')
            plt.ylabel('torque direction error (deg)')
            ##########
        return s[best_index], s1[best_index]
    
    def test_craft(craft, demands, vK=1, eps=0.1, maxI=50):
        total_error = 0
        for d in demands: 
            total_error += sum(optR(craft, d, vK, eps, maxI))
        print 'total error:', total_error
        print '='*80+'\n\n'
        plt.show()
        
#     test_craft(VTOL_Test, VTOL_Test_Bad_Demand, vK=0.1, eps=0.01, maxI=50)
#     test_craft(Hover_Test, Hover_Test_Bad_Demand+VTOL_Test_Bad_Demand, vK=0.3, eps=0.1, maxI=50)
    
    test_craft(VTOL_Test, VTOL_Test_Bad_Demand, vK=0.3, eps=0.01, maxI=50)
    
#     N = range(500)
#     np.random.seed(42)
#     random_tests = [vec(200*np.random.random()-100.0,
#                         200*np.random.random()-100.0,
#                         200*np.random.random()-100.0) 
#                     for _n in N]
#     E = []; A = []; X = []; Y = []; Z = [];
#     for d in random_tests:
#         e, a = optR(Hover_Test, d, vK=0.5, eps=0.1, maxI=30, output=False)
#         dm = abs(d)
#         E.append(e/dm*100); A.append(a);
#         X.append(d[0]/dm*100); Y.append(d[1]/dm*100); Z.append(d[2]/dm*100)
#          
#     plt.subplot(4,1,1)
#     plt.plot(A, E, 'o')
#     plt.xlabel('angle')
#     plt.ylabel('torque error (%)')
#     plt.subplot(4,1,2)
#     plt.plot(X, A, 'o')
#     plt.xlabel('x %')
#     plt.ylabel('angle')
#     plt.subplot(4,1,3)
#     plt.plot(Y, A, 'o')
#     plt.xlabel('y %')
#     plt.ylabel('angle')
#     plt.subplot(4,1,4)
#     plt.plot(Z, A, 'o')
#     plt.xlabel('z %')
#     plt.ylabel('angle')
#     plt.show()


def linalg_Attitude():
    
    def engines2a(engines):
        num_e = len(engines)
        a = np.array([[None]*num_e, [None]*num_e, [None]*num_e], dtype=float)
        for i, e in enumerate(engines):
            t = e.nominal_current_torque(1)
            a[0,i] = t[0]
            a[1,i] = t[1]
            a[2,i] = t[2]
        return a
    
    for d in VTOL_Test_Bad_Demand: 
        a = engines2a(VTOL_Test)
        x, r, R, s = np.linalg.lstsq(a, d.v)
        print "demand: %s\nlimits: %s\nresids: %s" % (d, x, np.matrix(a).dot(x)-d.v)
        print ''


def sim_VSpeed():
    sim1 = VSF_sim(10.0, 1, 10, 10.0)
    
    start_v = 2
    thrust = np.arange(55, 500, 50)
    
    def run_sim(c, n, vK=None):
        for t in thrust:
            sim1.run(start_v, maxV=1, thrust = t, vK=vK)
            sim1.plot(2,c,n)
            if start_v == 0: continue
            sim1.run(-start_v, maxV=1, thrust = t, vK=vK)
            sim1.plot(2,c,c+n)
    
    run_sim(2,1)
    run_sim(2,2, sim1.vK2)
    plt.show()
    
#==================================================================#
    
if __name__ == '__main__':
    sim_Attitude()
    #sim_VSpeed()