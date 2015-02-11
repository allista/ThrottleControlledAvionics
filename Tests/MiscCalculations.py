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
    if x > 1: return 1
    elif x < 0: return 0
    return x
#end def

def asymp01(x, k=1):
    return 1.0-1.0/(x/k+1.0); 

vclamp01 = np.vectorize(clamp01)


class sim(object):
    def __init__(self, K1, L1, K2, L2):
        self.K1 = K1
        self.L1 = L1
        self.K2 = K2
        self.L2 = L2
        
        self.upA  = 0.0
        self.upV  = 0.0
        self.upX  = 0.0
        
        self.T = []
        self.X = []
        self.V = []
        self.K = []
        self.F = []
    #end def
        
    @property
    def vK(self):
        E = self.maxV-self.upV
        if self.upV < self.maxV:
            return clamp01(E/2.0/(clampL(self.upA/self.K1+1.0, self.L1))**2)
        else: 
            return clamp01(E*self.upA/(clampL(-self.K2*E, self.L2))**2)
    
    def run(self, upV):
        t  = 0
        self.upA  = 0.0
        self.upV  = upV
        self.upX  = 100.0
        
        self.T = [t]
        self.V = [self.upV]
        self.X = [self.upX]
        self.K = [self.vK]
        self.F = [0.0]
        
        while t < self.t1:
            self.upA += self.thrust*self.K[-1] - 9.81
            self.upV += self.upA*self.dt
            self.upX += self.upV*self.dt
            t += self.dt
            self.T.append(t)
            self.F.append(self.thrust*self.K[-1] * self.dt)
            self.V.append(self.upV)
            self.X.append(self.upX)
            self.K.append(self.vK)
        
        print(("Start velocity: %f\n"
               "K1 %f; L1 %f; K2 %f L2 %f\n"
               "Fuel consumed: %f\n") 
              % (upV, self.K1, self.L1, self.K2, self.L2, sum(self.F)))
    #end def
    
    def plot(self):
        plt.subplot(2, 1, 1)
        plt.plot(self.T, self.V)
        plt.xlabel("time (s)")
        plt.ylabel("vertical speed (m/s)")
        
        plt.subplot(2, 1, 2)
        plt.plot(self.T, self.K)
        plt.xlabel("time (s)")
        plt.ylabel("thrust coefficient")
    #end def
    
    dt     = 0.005
    thrust = 150.0
    maxV   = 1.0
    t1     = 5.0
#end class

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
    def __init__(self, spec_torque, min_thrust=0.0, max_thrust=100.0):
        self.torque = spec_torque
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust 
        self.limit  = 1.0
        self.limit_tmp = 1.0
        self.current_torque = vec()
        
    def nominal_current_torque(self, K):
            return self.torque * lerp(self.min_thrust, self.max_thrust, K)
#end class

class PID(object):
    dt = 0.01
    
    def __init__(self, kp, ki, kd):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        
        self.value  = 0
        self.ierror = 0
        self.perror = 0
    #end def
        
    def update(self, err):
        self.ierror += err*self.dt
        act = self.kp*err + self.ki*self.ierror + self.kd*(err-self.perror)/self.dt
        self.perror = err
        return act
    #end def
    
    def sim(self, PV, SV, T):
        V = [0.0]
        for sv in SV[1:]:
            act = self.update(sv-V[-1])
            if np.isinf(act) or np.isnan(act): 
                act = sys.float_info.max
            V.append(PV+act)
        #plot
        plt.subplot(2,1,1)
        plt.plot(T, SV)
        plt.subplot(2,1,2)
        plt.plot(T, V)
#end clas
     

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
        if r > 0.98:
            sSV = rSV(sSV, 50)
            SV.append(sSV)
        elif r > 0.3:
            SV.append(rSV(sSV, 5))
        else: SV.append(SV[-1])
    
    pid1 = PID(0.1, 20.0, 0.0)
    pid2 = PID(0.1, 15.0, 0.0)
    
    pid1.sim(PV, SV, T)
    pid2.sim(PV, SV, T)
    plt.show()
    
    
def sim_VSpeed():
    sim1 = sim(10.0, 1, 10, 10.0)
    sim2 = sim(10.0, 1, 20, 10.0)
    
    start_v = 50
    
    sim1.run(start_v)
    sim1.plot()
    sim2.run(start_v)
    sim2.plot()
    plt.show()
    if start_v == 0: sys.exit()
    sim1.run(-start_v)
    sim1.plot()
    sim2.run(-start_v)
    sim2.plot()
    plt.show()
    
def sim_Attitude():
    def S(T, K, L=0): return vec.sum(Tk(T, K, L))
    def Sm(T, K, L=0): return abs(S(T,K))
    def Tk(T, K, L=0): return [t*clampL(k, L) for t, k in zip(T,K)]
    
    def opt(target, engines, eps):
        tm = abs(target)
        comp = vec()
        for e in engines:
            p = e.current_torque*target
            e.limit_tmp = -p/tm/abs(e.current_torque) if p < 0 else 0.0
            if e.limit_tmp > 0:
                comp += e.nominal_current_torque( e.limit)
        compm = abs(comp)
        if compm < eps: return False
        limits_norm = clamp01(tm/compm)
        for e in engines:
            e.limit *= 1.0 - clamp01(e.limit_tmp*limits_norm)
        return True
    
    def optR(engines, D=vec(), vK=1.0, eps=0.1, maxI=500, output=True):
        torque_clamp = vec6()
        torque_imbalance = vec()
        ti_min = vec(); ti_max = vec();
        for e in engines:
            e.limit = 1.0
            ti_min += e.nominal_current_torque(0)
            ti_max += e.nominal_current_torque(1)
        if ti_min > 0 and ti_max > 0: 
            vK = clampL(vK, clamp01(abs(ti_min)/abs(ti_max)))
        for e in engines:
            e.current_torque = e.nominal_current_torque(vK)
            torque_imbalance += e.current_torque
            torque_clamp.add(e.current_torque)
        _d = torque_clamp.clamp(D)
        if output:
            print 'vK: %f' % vK
            print 'Torque clamp:\n', torque_clamp
            print 'demand:        ', D
            print 'clamped demand:', _d
            print 'min TI: %s, max TI %s' % (ti_min, ti_max)
            print ('initial     %s, error %s, dir error %s' % 
                   (torque_imbalance, abs(torque_imbalance-_d), torque_imbalance.angle(_d)))
        s = []; s1 = []; i = 0;
        for i in xrange(maxI):
            s.append(abs(torque_imbalance-_d))
            s1.append(torque_imbalance.angle(_d) if abs(_d) > 0 else 0)
#             if len(s1) > 1 and s1[-1] < 55 and s1[-1]-s1[-2] > eps: break
            if s[-1] < eps or len(s) > 1 and abs(s[-1]-s[-2]) < eps: break
            mlim = max(e.limit for e in engines)
            for e in engines: e.limit = clamp01(e.limit/mlim)
            if not opt(D-torque_imbalance, engines, eps): break
            torque_imbalance = vec.sum(e.nominal_current_torque(vK * e.limit) for e in engines)
        if output:
            ##########
            print 'iterations:', i
            print 'limits:    ', list(e.limit for e in engines)
            print 'result      %s, error %s, dir error %s' % (torque_imbalance, s[-1], s1[-1])
            print 'engines:   ', list(e.nominal_current_torque(vK * e.limit) for e in engines)
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
        return s[-1], s1[-1]
    
    VTOL_Test = [
                    engine(vec(-3.4, -2.0, 0.0), min_thrust=0.0, max_thrust=250.0),
                    engine(vec(-3.4, 2.0, 0.0),  min_thrust=0.0, max_thrust=250.0),
                    engine(vec(3.5, -2.0, 0.0),  min_thrust=0.0, max_thrust=250.0),
                    engine(vec(1.5, 2.0, 0.0),   min_thrust=0.0, max_thrust=250.0),
                    engine(vec(1.3, 2.0, 1.6),   min_thrust=0.0,  max_thrust=20.0),
                    engine(vec(3.1, -2.0, -3.7), min_thrust=0.0,  max_thrust=20.0),
                    engine(vec(-3.1, -2.0, 3.7), min_thrust=0.0,  max_thrust=20.0),
                    engine(vec(-2.4, 2.0, -2.9), min_thrust=0.0,  max_thrust=20.0),
                 ]
    
    VTOL_Test_Bad_Demand = [
                            vec(0.0, 0.0, 0.0),
                            vec(100.0, 0.0, 0.0),
                            vec(0.0, 100.0, 0.0),
                            vec(0.0, 0.0, 100.0),
                                
                            vec(-1797.147, 112.3649, 80.1167), #Torque Error: 165.3752
                            vec(1327.126, -59.91731, 149.1847), #Torque Error: 229.8387
                            vec(107.5895, -529.4326, -131.0672), #Torque Error: 59.95838
                            vec(50.84914, -1.706408, 113.4622), #Torque Error: 25.10385
                            vec(-0.4953138, 0.2008617, 39.52808),
                            vec(14.88248, 0.9660782, -51.20171),
                            vec(-20.34281, -10.67025, 38.88113),
                            ]
    
    Hover_Test = [
                    engine(vec(-6.2, 6.4, 0.6), max_thrust=450),
                    engine(vec(-6.2, -6.4, -0.6), max_thrust=450),
                    engine(vec(3.9, 7.4, 0.6), max_thrust=450),
                    engine(vec(3.9, -6.4, -0.6), max_thrust=450)
                ]
    
    Hover_Test_Bad_Demand = [
                             vec(0.2597602, -5.2778279, 0.8444),
                             vec(0.9042788, -1.347212, 483.4898), #Torque Error: 483.4926kNm, 90deg
                             vec(-0.7012861, 0.2607862, -294.2217), #Torque Error: 339.2753kNm, 95.00191deg
                             vec(0.2597602, -0.2778279, 295.8444), #Torque Error: 341.1038kNm, 94.96284deg
                             ]
    
    for d in VTOL_Test_Bad_Demand: 
        optR(VTOL_Test, d, vK=0.37, eps=0.1, maxI=30)
    plt.show()
    print '='*80+'\n\n'
    
    for d in Hover_Test_Bad_Demand+VTOL_Test_Bad_Demand: 
        optR(Hover_Test, d, vK=0.37, eps=0.1, maxI=30)
    plt.show()
    print '='*80+'\n\n'

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
    
#==================================================================#
    
if __name__ == '__main__':
    sim_Attitude()