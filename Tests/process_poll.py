#!/bin/env python

import os
import pandas as pd
import matplotlib.pyplot as plt

dirname = os.path.join('TCA3_Poll')
data = os.path.join(dirname, 'TCA3 Gameplay (Responses) - Form responses 1.csv')

exclude = range(49,69)

df = pd.read_csv(data)

ts = pd.to_datetime(df.Timestamp)
ts = pd.Series([30]).append(pd.Series(pd.Timedelta(ts[i]-ts[i-1]).seconds if ts[i] > ts[i-1] else 0 for i in xrange(1, len(ts)))).reset_index(drop=True)
ts.plot(kind='area')
plt.show()

ax = ts.hist(bins=400)
plt.gcf().autofmt_xdate()
plt.show()

print ts[exclude]
print df.Timestamp[exclude]
print df.loc[~(df.index.isin(exclude))].iloc[:, [1,3,4]]

df1 = df.loc[~(df.index.isin(exclude))].iloc[:, [1,3,4]].apply(pd.value_counts)
for i, c in enumerate(df1.columns):
    plt.subplot(3, 1, 1+i)
    df1[c].dropna().plot(kind='bar', legend=True)
    plt.xticks(rotation=0)
plt.show()
