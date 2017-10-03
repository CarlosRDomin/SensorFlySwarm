# Example Usage:
# python sg.py position.dat 7 2

import math
import sys

import numpy as np
import pylab as py

def sg_filter(x, m, k=0):
    """
    x = Vector of sample times
    m = Order of the smoothing polynomial
    k = Which derivative
    """
    mid = len(x) / 2
    a = x - x[mid]
    expa = lambda x: map(lambda i: i**x, a)
    A = np.r_[map(expa, range(0,m+1))].transpose()
    Ai = np.linalg.pinv(A)

    return Ai[k]

def smooth(x, y, size=5, order=2, deriv=0):

    if deriv > order:
        raise Exception, "deriv must be <= order"

    n = len(x)
    m = size

    result = np.zeros(n)

    for i in xrange(m, n-m):
        start, end = i - m, i + m + 1
        f = sg_filter(x[start:end], order, deriv)
        result[i] = np.dot(f, y[start:end])

    if deriv > 1:
        result *= math.factorial(deriv)

    return result

def plot(t, plots):
    n = len(plots)

    for i in range(0,n):
        label, data = plots[i]

        plt = py.subplot(n, 1, i+1)
        plt.tick_params(labelsize=8)
        py.grid()
        py.xlim([t[0], t[-1]])
        py.ylabel(label)

        py.plot(t, data, 'k-')

    py.xlabel("Time")

def create_figure(size, order):
    fig = py.figure(figsize=(8,6))
    nth = 'th'
    if order < 4:
        nth = ['st','nd','rd','th'][order-1]

    title = "%s point smoothing" % size
    title += ", %d%s degree polynomial" % (order, nth)

    fig.text(.5, .92, title,
             horizontalalignment='center')

def load(name):
    f = open(name)
    dat = [map(float, x.split(' ')) for x in f]
    f.close()

    xs = [x[0] for x in dat]
    ys = [x[1] for x in dat]

    return np.array(xs), np.array(ys)

def plot_results(data, size, order):
    # t, pos = load(data)
    l = np.load(data)
    t = l[:, 0]
    pos = l[:, 1]
    params = (t, pos, size, order)

    plots = [
        ["Position",     pos],
        ["Velocity",     smooth(*params, deriv=1)],
        ["Acceleration", smooth(*params, deriv=2)]
    ]

    create_figure(size, order)
    plot(t, plots)

if __name__ == '__main__':
    data = sys.argv[1]
    size = int(sys.argv[2])
    order = int(sys.argv[3])

    plot_results(data, size, order)
    py.show()
