import matplotlib.pyplot as plt
import numpy as np

plt.rc('text', usetex=True)

p1 = np.arange(-30, 30, 0.5)

plt.plot(p1, 2 - p1)
plt.plot(p1, 1 - 0.25*p1**2)
plt.fill_between(p1, 2-p1, 1-0.25*p1**2)

plt.grid(True)
plt.xlabel('p_1', fontsize=20)
plt.ylabel('p_2', fontsize=20)
plt.title('$0 < k < 1$', fontsize=20)
plt.xlim((0,30))
plt.axis('equal')
plt.show()