import matplotlib.pyplot as plt
import numpy as np

plt.rc('text', usetex=True)

p1 = np.arange(0, 50, 0.5)

plt.plot(p1, 2 + p1)
plt.plot(p1, 1 - 0.25*p1**2)
plt.fill_between(p1, 2+p1, 1-0.25*p1**2, color='none',hatch="/", edgecolor='blue')

plt.grid(True)
plt.xlabel('p_1', fontsize=20)
plt.ylabel('p_2', fontsize=20)
plt.title('$0 < k < 1$', fontsize=20)
plt.xlim((0,30))
plt.axis('equal')

plt.text(5,4,'$0 < k < 1$', fontsize=30)

plt.show()