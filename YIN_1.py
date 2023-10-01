# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import math as m
import matplotlib.pyplot as plt
import numpy as np
import random as rand
from numpy import pi 
from numpy.linalg import inv

x=[1,2,3]

y=np.array([[1] ,[2] ,[3] ,[4]])




"""

print(y)

"""

z=(2*(512*m.cos(pi/3)+13**2))/127

"""
3<4
Out[58]: True

3<=3
Out[59]: True

1!=2
Out[60]: True

1==2
Out[61]: False

"""

A = np.array([[1,2,3],[4,5,6]])
B = np.array([[1,2],[3,4],[5,6]])
C = np.dot(A,B)
D = np.dot(B,A)
"""

print(A)
print(B)
print(C)

"""

A = np.array([[1,2],[3,4]])
B = np.array([[5,6],[7,8]])
C = A @ B
D = A * B

print("------------------------------------")


M = np.array([[1,2],[3,4]])


#print(M.shape)
"""

D=np.linalg.inv(M)
print(D)

E=np.transpose(M)
print(E)


D = A.T (ou np.transpose())

"""

A=np.ones((3,3),dtype=int)
B=np.zeros(3)
C=np.random.rand(3,3,2  )

#print(A)
#print(B)
#print(C)




    