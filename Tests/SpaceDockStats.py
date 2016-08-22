#!/bin/env python
# coding=utf-8

from __future__ import print_function

import urllib

import math
import pandas as pd
from collections import Counter
from datetime import datetime, timedelta

if __name__ == '__main__':
    date_fmt = '%Y-%m-%d %H:%M:%S.%f'
    filename = u'downloads.csv'
    urllib.urlretrieve('http://spacedock.info/mod/198/Throttle%20Controlled%20Avionics/stats/downloads', filename)
    downloads = pd.read_csv(filename)
    #compute counts and time deltas
    count = Counter()
    period = dict()
    for row in downloads.iterrows():
        r = row[1]
        v = str(r['Mod Version']).lstrip('v')
        t = datetime.strptime(str(r['Date'].strip()), date_fmt)
        p = period.get(v)
        if p:
            if p[0] > t: p[0] = t
            elif p[1] < t: p[1] = t
        else:
            period[v] = [t, t]
        count[v] += row[1]['Downloads']
    #compose report
    report = []
    header = ['Version', 'Downloads / hours = ratio']
    maxv = max((len(v) for v in count.keys()+[header[0]]))
    maxd = max(len(str(d)) for d in count.values())
    report.append('%s%s: %s' % (header[0], ' '*(maxv-len(header[0])+1), header[1]))
    for v in sorted(count.keys()):
        d = count[v]
        p = period[v]
        delta = math.floor((p[1]-p[0]).total_seconds()/3600.0)
        report.append('%s%s: %d%s/ %d = %f' %
                      (v,
                       ' ' * (maxv - len(v) + 1), d,
                       ' ' * (maxd - len(str(d)) + 1), delta, d/float(delta)))
    print('\n'.join(report))
