from __future__ import print_function
from scipy.optimize import curve_fit
import pandas as pd
import matplotlib.pyplot as plt

from common import gamefile

if __name__ == '__main__':
    df = pd.read_csv(gamefile('Tardigrade.AttitudeControl.csv'),
                              names=('axis', 'throttle',
                                     'AAx','AAy','AAz',
                                     'ODx', 'ODy', 'ODz',
                                     'kDx', 'kDy', 'kDz',
                                     'AAf'))

    print(df.to_string())
    
    def func(aa, a,b):
        return a/(b+aa)
        
    pitch = df[df.axis == 1]
    roll = df[df.axis == 0].sort_values('AAy')
        
    roll_opt, pcov = curve_fit(func, roll.AAy, roll.AAf)
    print('roll', roll_opt)
        
    pitch_opt, pcov = curve_fit(func, pitch.AAx, pitch.AAf)
    print('pitch', pitch_opt)

    ax = roll.plot.scatter('AAy', 'AAf', c='r')
    ax.plot(roll.AAy, func(roll.AAy, *roll_opt), 'r--')
    pitch.plot.scatter('AAx', 'AAf', ax=ax)
    ax.plot(pitch.AAx, func(pitch.AAx, *pitch_opt), '--')

    plt.show()
