'''
Created on Feb 21, 2016

@author: player
'''

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.fftpack import fft
    
def plt_show_maxed():
    plt.tight_layout(pad=0, h_pad=0, w_pad=0)
    plt.subplots_adjust(top=0.99, bottom=0.03, left=0.05, right=0.99, hspace=0.25, wspace = 0.1)
    mng = plt.get_current_fig_manager()
    mng.window.showMaximized()
    plt.show()

if __name__ == '__main__':
    data = 'oscillation.csv'
    df = pd.read_csv(data, names=('tag', 
                                  'UT',
                                #HSC
#                                 'Nx', 'Ny', 'Nz',
#                                 'fTx', 'fTy', 'fTz',
#                                 'Tx', 'Ty', 'Tz',
#                                 ))
                                #attitude
                                'ErrDeg',
                                'Sx', 'Sy', 'Sz',
                                'AVx', 'AVy', 'AVz',
                                'dSx', 'dSy', 'dSz',
                                'Px', 'Py', 'Pz',
                                ))
    df.UT -= df.UT[0]
    del df['tag']
    #dt
    dt = df.UT[1]-df.UT[0]
    rate = 1.0/dt
    #plot the data
    trange = [24,55]
    df1 = df[df.UT > trange[0]][df.UT < trange[1]]
    cols = df1.columns[3:10:3]
    nplots = len(cols)
    for i, c in enumerate(cols):
        plt.subplot(nplots,1,1+i)
        plt.plot(df1.UT, df1[c])
        plt.ylabel(c)
        plt.grid()
    plt_show_maxed()
    #fft
    
#     x = np.arange(0,1, 0.001)
#     y = x**0.40
#     plt.plot(x,y)
#     plt.grid()
#     plt.xticks(np.arange(0,1,0.04), np.arange(0,1,0.04)*180)
#     plt.show()
    
#     def ffreq(s):
#         rlen = int(len(s)/2)
#         fftres = fft(s)
#         freqs = np.abs(fftres[:rlen])
# #         plt.plot(np.array(range(rlen))*rate/rlen/2, freqs)
# #         plt.show()
#         ffbin = np.argmax(freqs)
#         print 'Fundamental Frequency: %.2fHz, amplitude is %d' % (ffbin*rate/rlen/2, np.amax(freqs))
#     
#     window = 30
#     w = int(window/dt)
#     for c in df.columns[1:]:
#         print
#         print '-'*80
#         print '%s: window: %d = %.2fs' % (c, w, window) 
#         lasti = 0
#         for i in xrange(w, w*(len(df[c])/w+2), w):
#             print '[%.2fs, %.2fs)' % (lasti*dt, i*dt) 
#             ffreq(df[c][lasti:i])
#             lasti = i