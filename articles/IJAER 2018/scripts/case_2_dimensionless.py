import matplotlib.pyplot as plt
import numpy as np

plt.rc('text', usetex=True)


delta = 0.025
p1 = np.arange(-3, 0, delta)
p2 = np.arange( 0, 5, delta)

P1, P2 = np.meshgrid(p1,p2)

Z1 = 0.5*P1 + (0.25*P1**2 + P2 - 1)**0.5

V = np.arange(0,1.1,0.1)
CS = plt.contour(P1,P2,Z1,V,colors='black')
plt.clabel(CS,CS.levels)


p1 = np.arange(0,5,delta)
p2 = np.arange(-3,0,delta)

P1, P2 = np.meshgrid(p1,p2)

Z1 = 0.5*P1 - (0.25*P1**2 + P2 - 1)**0.5

V = np.arange(0,1.1,0.1)
CS = plt.contour(P1,P2,Z1,V,colors='black')
plt.clabel(CS,CS.levels)

#plt.plot(p1, 2 - p1)
#plt.plot(p1, 1 - 0.25*p1**2)
p1 = np.arange(2,5,delta)
plt.fill_between(p1, 2-p1, 0, color='grey')
p1 = np.arange(-3,0,delta)
plt.fill_between(p1, 2-p1, 1, color='grey')

plt.grid(True)
plt.xlabel('p_1', fontsize=20)
plt.ylabel('p_2', fontsize=20)
plt.title('$0 < k < 1$', fontsize=20)
plt.xlim((0,30))
plt.axis('equal')

#plt.text(0,1.25,'$0 < k < 1$', fontsize=30)

plt.show()