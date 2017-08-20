from common import lerp, dt, clamp01
from KSPUtils import Part, SearchTerm

class Engine(object):
    def __init__(self, maxThrust, acceleration=0, deceleration=0):
        self.name = 'Engine'
        self.maxThrust = maxThrust
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.instant = acceleration == 0 and deceleration == 0
        self.limit = 1
        self.lever = 1
        self.thrust = 0
        self.torque = 0

    def __str__(self):
        s = [self.name, 'maxThrust: %f' % self.maxThrust]
        if self.acceleration > 0:
            s.append('accelerationSpeed: %f' % self.acceleration)
        if self.deceleration > 0:
            s.append('decelerationSpeed: %f' % self.deceleration)
        return '\n'.join(s)

    def update(self):
        request = self.maxThrust*self.limit
        delta = request-self.thrust
        if delta > 0:
            if self.acceleration > 0:
                self.thrust = lerp(self.thrust, request, self.acceleration * dt)
            else: self.thrust = request
        elif delta < 0:
            if self.deceleration > 0:
                self.thrust = lerp(self.thrust, request, self.deceleration * dt)
            else: self.thrust = request
        self.torque = self.thrust*self.lever

    module_term = SearchTerm('PART/MODULE:ModuleEngines.*/')

    def clone(self):
        e = Engine(self.maxThrust, self.acceleration, self.deceleration)
        e.limit = self.limit
        e.thrust = self.thrust
        e.lever = self.lever
        e.torque = self.torque
        return e

    @classmethod
    def from_file(cls, path):
        p = Part.LoadFromFile(path)
        if p is None: return None
        p = list(p)[0]
        engine = cls.module_term.select(p)
        if not engine: return None
        engine = engine[0]
        try:
            maxThrust = float(engine.GetValue('maxThrust'))
            useResponse = bool(engine.GetValue('useEngineResponseTime'))
            accelSpeed = 0
            decelSpeed = 0
            if useResponse:
                accelSpeed = float(engine.GetValue('engineAccelerationSpeed'))
                decelSpeed = float(engine.GetValue('engineDecelerationSpeed'))
        except Exception as e:
            print str(e)
            return None
        e = Engine(maxThrust, accelSpeed, decelSpeed)
        e.name = p.name
        return e
