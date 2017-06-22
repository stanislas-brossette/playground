#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sympy import *
x, y = symbols('x y')
# a = Integral(cos(x)*exp(x),x)
# Eq(a, a.doit())
expr = x + 2*y
print integrate(sin(x**2), (x, -oo, oo));
A = Matrix([[0,1,0][0,0,1][0,x**2,0]])
print A

