"""


"""

import math as m
import matplotlib.pyplot as plt
import numpy as np
import random as rand
from numpy import pi 
from numpy.linalg import inv
import scipy.optimize as resol


delta_t = 0.01
nombre_echantillons = 1000
temps = np.arange(0,nombre_echantillons)*delta_t
signal = 1.9*np.sin(5*temps)+ 1.5*np.sin(11*temps)+1.7*np.sin(2.3*temps)
signal = signal + np.sqrt(0.3)*np.random.randn(np.size(temps))




def recursive(nombre_echantillons,a,signal):


    y = np.zeros(nombre_echantillons)


# Filtrage du signal
    for n in range(1, nombre_echantillons):
        y[n] = a * signal[n]+(1-a)* y[n-1]
    
    plt.figure(figsize=(10,6))
    plt.subplot(2,1,1)
    plt.plot(temps,signal)
    plt.xlabel('Temps')
    plt.ylabel('Amplitude')
    plt.legend('Signal entrée')

    plt.subplot(2,1,2)
    plt.plot(temps,y)
    plt.xlabel('Temps')
    plt.ylabel('Amplitude')
    plt.legend('Signal sortie')

    plt.tight_layout()
    
recursive(1000,0.1,signal)

def Filtre_Passebas(*argument):
    y = np.zeros(nombre_echantillons)
    entree = argument[0]
    if len(argument) == 1:
        a = 0.4
    else:
        for x in range(1,len(argument)):
            a = argument[x]
            
            for n in range(nombre_echantillons):
                y[n] = a * entree[n]+(1-a)* y[n-1]
                
    plt.plot(temps,y)
    
    
            
Filtre_Passebas(signal,0.7)     
    
    
    
    

"""
Partie 2.3
"""

# fonction
def myFun(*args):
  for a in args:
   print(a)
   print(type(a))
   

myFun('Hello', 'Welcome', 'to', 'GeeksforGeeks',1,'2')


"""

def print_func(x, *args, **kwargs):
    print(x)
    print(args)
    print(kwargs)

print_func(1, 2, 3, 4, y=1, a=2, b=3, c=4)


Résultat:

 1
(2, 3, 4)
{'y': 1, 'a': 2, 'b': 3, 'c': 4}


"""

"""
Partie 2.5
"""


def f(x):
        return 0.8-(np.exp((-(x-3)**2-5)/100)-np.sin(pi*((x/30)+1)))
plt.figure()
x1 = np.linspace(-20,20,41)
plt.plot(x1,f(x1))
print("f(x)=0 pour x = ",resol.fsolve(f,20))
"""
Ici 20 c'est le point qu'on pense que f(20) = 0 mais pas précisé, il va continuer de chercher 
le point x : f(x) = 0
"""




minimum = resol.fmin(f,10)
print("f(x) minimum pour x = ", minimum[0])




"""
Partie 3
"""

























