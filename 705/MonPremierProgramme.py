#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 13:30:54 2023

@author: zeyu.yin@etu.umontpellier.fr
"""

import math as m
import matplotlib.pyplot as plt
import numpy as np
import random as rand
from numpy import pi 
from numpy.linalg import inv

delta_t = 0.01
nombre_echantillons = 1000
temps = np.arange(0,nombre_echantillons)*delta_t
signal = 1.9*np.sin(5*temps)+ 1.5*np.sin(11*temps)+1.7*np.sin(2.3*temps)
signal = signal + np.sqrt(0.3)*np.random.randn(np.size(temps))



a=0.2

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