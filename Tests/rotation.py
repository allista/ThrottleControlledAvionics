#!/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    def RotationTime3Phase(angle, aa, accel_part, throttle):
        ak2 = 2*accel_part*angle
        return (angle+ak2)/np.sqrt(ak2*aa*throttle)
        
    acc_part = np.linspace(0.001, 0.5, 1000)
    plt.plot(acc_part, RotationTime3Phase(np.pi, np.pi/10, acc_part, 1), label='thr=1')
    plt.plot(acc_part, RotationTime3Phase(np.pi, np.pi/10, acc_part, 0.5), label='thr=0.5')
    plt.plot(acc_part, RotationTime3Phase(np.pi, np.pi/10, acc_part, 0.2), label='thr=0.2')
    plt.legend()
    plt.show()
    
