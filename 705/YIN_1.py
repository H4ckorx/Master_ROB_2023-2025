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
C=np.random.rand(3,3,2)
#3个3*2的矩阵

print(C)
#print(A)
#print(B)
#print(C)

"""
EXercice

"""




# Nombre de points
nbp=50
# Création vecteur x vertical v1
x=np.linspace(0,10,nbp)
x.shape=(nbp,1)

# Création du vecteur y par l'équation de la droite
a=2;b=-3
y=a*x+b
# Ajout de bruit
noise = (np.random.randn(np.size(y))*np.sqrt(2))
#
"""
np.random.randn生成 -2到2之间

np.random.rand生成  0到1之间

"""
noise.shape = (nbp,1)
yn = y + noise

"""
size的用法
import numpy as np
X=np.array([[1,2,3,4],
              [5,6,7,8],
              [9,10,11,12]])
 
number=X.size  # 计算 X 中所有元素的个数
X_row=np.size(X,0)  #计算 X 的行数
X_col=np.size(X,1)  #计算 X 的列数
 
print("number:",number)
print("X_row:",X_row)
print("X_col:",X_col)
 
<<
number: 12
X_row: 3
X_col: 4

"""



"""
shape的用法
import numpy as np
X=np.array([[1,2,3,4],
              [5,6,7,8],
              [9,10,11,12]])
 
X_dim=X.shape  # 以元组形式，返回数组的维数
print("X_dim:",X_dim)
print(X.shape[0])  # 输出行的个数
print(X.shape[1])  #输出列的个数
 
<<
X_dim: (3, 4)
3
4


"""
# Vecteur de uns vertical v2
o=np.ones(nbp)[np.newaxis]

"""
np.newaxis 放在哪个位置，就会给哪个位置增加维度

x[:, np.newaxis] ，放在后面，会给列上增加维度
x[np.newaxis, :] ，放在前面，会给行上增加维度

"""
#print(o)
o = o.T
# Création des matrices A et B
A=np.hstack((x,o))
"""
np.hstack将参数元组的元素数组按水平方向进行叠加
import numpy as np
 
arr1 = np.array([[1,3], [2,4] ])
arr2 = np.array([[1,4], [2,6] ])
res = np.hstack((arr1, arr2))
 
print (res)
 
#
[[1 3 1 4]
 [2 4 2 6]]

"""
B=np.array(yn)
print(yn)
# Calcul des coefficients
c=inv(A.T@A)@A.T@B
print(c)
#print(noise)
# Droite approchée
yr = c[0]*x+c[1]
# Affichage graphique
plt.plot(x,yr);
plt.plot(x,yn,'.')
