#!/usr/bin/env python
"""
Estimate the timing of functions in the warping module.
"""

import warping, random, time

def time_ss(n):
    ss = []
    for i in range(n):
        ss.append(random.randint(0,1))

    start = time.time()
    warping.set_snapshot(0, ss)
    return time.time() - start

def time_cv(n):
    cv = []
    for i in range(n):
        cv.append(random.randint(0,1))

    start = time.time()
    warping.compute_home_vector(0, cv)
    elapsed = time.time() - start
    return time.time() - start

warping.init(1)

trials = 10

for n in range(0, 100, 10):
    print "n: " + str(n)

    avg_ss = 0
    avg_cv = 0
    for i in range(trials):
        tss = time_ss(n)
        print "ss elapsed: " + str(tss)
        tcv = time_cv(n)
        print "cv elapsed: " + str(tcv)
        avg_ss += tss
        avg_cv += tcv
    print "ss elapsed (average): " + str(avg_ss / trials)
    print "cv elapsed (average): " + str(avg_cv / trials)
