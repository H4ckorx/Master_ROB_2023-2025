import math as m
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

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
noise.shape = (nbp,1)
yn = y + noise
# Vecteur de uns vertical v2
o=np.ones(nbp)[np.newaxis]
o = o.T
# Création des matrices A et B
A=np.hstack((x,o))
B=np.array(yn)
# Calcul des coefficients
c=inv(A.T@A)@A.T@B
print(c)
# Droite approchée
yr = c[0]*x+c[1]
# Affichage graphique
#plt.plot(x,y);
#plt.plot(x,yn,'.');
#plt.plot(x,yr,color='red');
# Calcul de l'erreur d'approximation
e=((yr-y)**2)
e2=sum(e)/nbp
e3=e.T@e
print("Erreur:")
print(" a) par moyenne des composantes : {}".format(e2[0]))
print(" b) par calcul du module du vecteur erreur : {}".format(e3[0][0]))