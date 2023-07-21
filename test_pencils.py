import numpy as np
from common import EllipticalBarrier, BarrierGrid
import matplotlib.pyplot as plt

barrier1 = EllipticalBarrier(shape=[2,1])
barrier2 = EllipticalBarrier(shape=[2,1])

barrier_grid = BarrierGrid( barriers = [barrier1, barrier2] )
# barrier_grid.update_barrier(0, np.random.rand(3)-0.5)
# barrier_grid.update_barrier(1, 10*np.random.rand(3)-5)

barrier_grid.update_barrier(0, [ 2.2, 1, 0 ])
barrier_grid.update_barrier(1, [ 2, 3, 0 ])

num_barriers = len(barrier_grid.barriers)

barrier_value, barrier_gradient, neighbor_barrier_gradient, opt_ellipse = barrier_grid.compute_barrier(0,1)

print("Barrier value = " + str(barrier_value))
print("Barrier gradient = " + str(barrier_gradient))

print("Ellipse point = " + str(opt_ellipse))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('Ellipse distance')
ax.set_aspect('equal', adjustable='box')

resolution = 50
t_series = np.linspace(0, 2*np.pi, resolution)
xdata1, ydata1 = np.zeros(resolution), np.zeros(resolution)
xdata2, ydata2 = np.zeros(resolution), np.zeros(resolution)
for k in range(resolution):
    ellipse_pt1 = barrier1.ellipse(t_series[k])
    ellipse_pt2 = barrier2.ellipse(t_series[k])
    xdata1[k], ydata1[k] = ellipse_pt1[0], ellipse_pt1[1]
    xdata2[k], ydata2[k] = ellipse_pt2[0], ellipse_pt2[1]

ellipse1, = ax.plot(xdata1,ydata1,lw=1,color='blue')
ellipse2, = ax.plot(xdata2,ydata2,lw=1,color='blue')

p1 = barrier1.center
p2 = barrier2.center

ax.plot(opt_ellipse[0], opt_ellipse[1],lw=1,marker='o',color='red')

plt.show()
