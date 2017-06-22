#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

priceN3 = [1.55 , 1.64 , 2.66 , 3.14 , 4.92 , 6.81 , 7.2  , 7.63 , 7.75 , 7.83 , 8.65 , 9.47 , 9.62 , 10.01, 10.09, 10.27, 10.4 , 11.1 , 11.29, 11.49, 13.08]
paidN3 = [0.64, 0.68, 1.1, 1.3, 2.03, 2.81, 2.97, 3.15, 3.2, 3.23, 3.57, 3.91, 3.97, 4.18, 4.26, 4.44, 4.57, 5.27, 5.46, 5.66, 7.25]
extPriceN3 = [0,1.55]
extPaidN3 = [0,0.64]

priceN2 = [6.85  , 7.52  , 7.77  , 8.49  , 9.37  , 9.49  , 9.49  , 9.72 , 10.05 , 10.21 , 10.29 , 10.33 , 10.43 , 10.75 , 11.75 , 12.19]
paidN2 = [1.92, 2.1 , 2.17, 2.38, 2.62, 2.66, 2.66, 2.72, 2.9 , 3.06, 3.14, 3.18, 3.28, 3.6 , 4.6 , 5.04]
extPriceN2 = [0, 6.85]
extPaidN2 = [0, 1.92]

priceN1 = [ 5.66, 6.86 , 7.09 , 9.27 , 9.55 , 9.87 , 10.17, 10.59, 12.52]
paidN1 = [1.36, 1.64, 1.7, 2.22, 2.29, 2.36, 2.62, 3.04, 4.97]
extPriceN1 = [0, 5.66]
extPaidN1 = [0,1.36]

def computePriceLaw(price, paid, group):
    a0 = paid[0]/price[0]
    b0 = 0.0
    i=0;
    while price[i] < 10.0:
        i = i+1;
    a1 = (paid[i+1]-paid[i])/(price[i+1]-price[i])
    b1 = paid[i]-price[i]*a1;
    
    print "price law for ", group, ":"
    print "  if price < 10E: ", "paid = price * " , a0
    print "  else: ", "paid = price * " , a1, "+", b1


computePriceLaw(priceN3, paidN3, "N3")
computePriceLaw(priceN2, paidN2, "N2")
computePriceLaw(priceN1, paidN1, "N1")

fig = plt.figure()
ax = fig.add_subplot(111)
plt.plot(extPriceN3, extPaidN3, 'b--')
plt.plot(extPriceN2, extPaidN2, 'g--')
plt.plot(extPriceN1, extPaidN1, 'r--')
plt.plot(priceN1, paidN1, 'r', label='N1')
plt.plot(priceN2, paidN2, 'g', label='N2')
plt.plot(priceN3, paidN3, 'b', label='N3')
plt.legend()
plt.grid(True)
ax.set_xlabel('total value')
ax.set_ylabel('price paid')
plt.show()

