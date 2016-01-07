#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

mass = 1.
inertias = np.eye(3)
d1 = 1
d2 = 9
d3 = 25
inertias = np.diag([1, 2, 3])

xx, yy, zz = np.meshgrid(*[np.linspace(-2, 2, 300)]*3)

def e3(d1, d2, d3, b1, b2, b3):
  res = d1/3 + d2/3 + d3/3 - ((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**3/27 - ((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**3/27 - ((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**2 + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)**3)**(1/2) + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**(1/3)/2 + (3**(1/2)*(((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**3/27 - ((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**3/27 - ((b1**2 + b2**2 + b3**2 + d1 + d2 + d3)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**2 + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)**3)**(1/2) + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**(1/3) + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)/((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**3/27 - ((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**3/27 - ((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**2 + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)**3)**(1/2) + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**(1/3))*1j)/2 + b1**2/3 + b2**2/3 + b3**2/3 + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (b1**2 + b2**2 + b3**2 + d1 + d2 + d3)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)/(2*((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**3/27 - ((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**3/27 - ((d1 + d2 + d3 + b1**2 + b2**2 + b3**2)*(d1*d2 + d1*d3 + d2*d3 + b1**2*d2 + b2**2*d1 + b1**2*d3 + b3**2*d1 + b2**2*d3 + b3**2*d2))/6 + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**2 + ((d1*d2)/3 + (d1*d3)/3 + (d2*d3)/3 - (d1 + d2 + d3 + b1**2 + b2**2 + b3**2)**2/9 + (b1**2*d2)/3 + (b2**2*d1)/3 + (b1**2*d3)/3 + (b3**2*d1)/3 + (b2**2*d3)/3 + (b3**2*d2)/3)**3)**(1/2) + (d1*d2*d3)/2 + (b1**2*d2*d3)/2 + (b2**2*d1*d3)/2 + (b3**2*d1*d2)/2)**(1/3))
  return res.real > b1**2+b2**2+b3**2


correct_points = (e3(d1, d2, d3, xx, yy, zz) >= xx**2+yy**2+zz**2)

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
