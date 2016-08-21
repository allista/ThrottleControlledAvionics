#!/bin/env python
# coding=utf-8

from __future__ import print_function

import urllib
import pandas as pd
from collections import Counter

if __name__ == '__main__':
    filename = u'downloads.csv'
    urllib.urlretrieve('http://spacedock.info/mod/198/Throttle%20Controlled%20Avionics/stats/downloads', filename)
    downloads = pd.read_csv(filename)
    count = Counter()
    for row in downloads.iterrows():
        v = str(row[1]['Mod Version']).lstrip('v')
        count[v] += row[1]['Downloads']
    report = []
    maxl = max((len(v) for v in count.keys()))
    for v in sorted(count.keys()):
        report.append('%s%s: %d' % (v, ' '*(maxl-len(v)+1), count[v]))
    print('\n'.join(report))
