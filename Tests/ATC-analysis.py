from scipy.signal import correlate
import numpy
from matplotlib import pyplot as plt

from common import gamefile
from analyze_csv import analyzeCSV

if __name__ == '__main__':
    df = analyzeCSV(gamefile('Jet_Hangar_Test.AttitudeControl.csv'),
                   ['err_X', 'err_Y', 'err_Z',
                    'atPID_X', 'atPID_Y', 'atPID_Z',
                    'avEr_X', 'avEr_Y', 'avEr_Z',
                    'av_X', 'av_Y', 'av_Z',
                    'aa_X', 'aa_Y', 'aa_Z',
                    'pitch', 'roll', 'yaw',],
                   ['err_X',
                    'aa_X',
                    'av_X',
                    'avEr_X',
                    'pitch', ],
                    # region=(2000,)
                   )

    # nsamples = len(df.av_X)
    # xcorr = correlate(df.MaxAA_X, df.pitch)[nsamples-1:]
    # dt = numpy.arange(nsamples)
    #
    # plt.plot(dt, xcorr)
    # plt.show()
    #
    # recovered_time_shift = dt[xcorr.argmax()]
    # print "time shift: %d" % (recovered_time_shift)
