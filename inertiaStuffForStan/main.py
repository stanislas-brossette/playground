#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

mass = 1.
inertias = np.eye(3)
inertias = np.diag([1, 2, 3])

xx, yy, zz = np.meshgrid(*[np.linspace(-2, 2, 100)]*3)

def ineq1(xx, yy, zz, inertias):
  return yy**2 + zz**2 <= inertias[0, 0]/mass

def ineq2(xx, yy, zz, inertias):
  return xx**2 + zz**2 <= inertias[1, 1]/mass

def ineq3(xx, yy, zz, inertias):
  return xx**2 + yy**2 <= inertias[2, 2]/mass

correct_points = np.logical_and(np.logical_and(ineq1(xx, yy, zz, inertias),
                                               ineq2(xx, yy, zz, inertias)),
                                ineq3(xx, yy, zz, inertias))

x, y, z = xx[correct_points], yy[correct_points], zz[correct_points]
nx, ny, nz = xx[~correct_points], yy[~correct_points], zz[~correct_points]

fig = plt.figure()

ax = fig.add_subplot('311')
ax.set_aspect('equal')
ax.set_title('xy')
#ax.scatter(nx, ny, color='red', alpha=0.1)
ax.scatter(x, y, color='blue')

ax = fig.add_subplot('312')
ax.set_aspect('equal')
ax.set_title('xz')
#ax.scatter(nx, nz, color='red', alpha=0.1)
ax.scatter(x, z, color='blue')

ax = fig.add_subplot('313')
ax.set_aspect('equal')
ax.set_title('yz')
#ax.scatter(ny, nz, color='red', alpha=0.1)
ax.scatter(y, z, color='blue')

plt.show()
