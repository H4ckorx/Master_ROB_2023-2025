import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

x = np.arange(-4,4,0.2) ; y = x
X, Y = np.meshgrid(x, y)

# # Sinus cardinal

# z = np.sinc(X)*np.sinc(Y)

# fig7 = plt.figure(7)
# ax = plt.axes(projection='3d')
# ax.plot_surface(X, Y, z, cmap='viridis')

# fig8 = plt.figure(8)
# bx = plt.axes(projection='3d')
# bx.plot_wireframe(X, Y, z, color='black')

# # Gaussienne

z = np.exp(-X**2-Y**2)

# fig9 = plt.figure(9)
# cx = plt.axes(projection='3d')
# cx.plot_surface(X, Y, z, cmap='jet')
# cx.view_init(90,-90,0)

fig10 = plt.figure(10)
plt.contourf(X, Y, z, 15, cmap='jet')
