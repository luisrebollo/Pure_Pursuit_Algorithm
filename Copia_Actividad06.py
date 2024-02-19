#Luis Enrique Camanos Rebollo
import numpy as np
import math as mt
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
via_points = ([0,0], [1,2], [3,4],[4,5],[7, 3],[5,2],[4,0])
#via_points = ([0,0], [1,2], [3,1],[5,4],[7, 2],[8,5],[10,0])
x_points, y_points = zip(*via_points)
ti = np.arange(len(via_points))
cs_x = CubicSpline(ti, x_points)
cs_y = CubicSpline(ti, y_points)
num_points_between_via_points = 25
t_interpolated = np.linspace(0, len(via_points) - 1, num_points_between_via_points * (len(via_points) - 1))
x_interpolated = cs_x(t_interpolated)
y_interpolated = cs_y(t_interpolated)
#3
length_wheel = 0.8
t = 10
step = len(x_interpolated) - 1
time = np.linspace(0,t,step)
dt = t / step
distancia = 0.1
# 7 0 20
Kvp = 4
Kip = 2
Ktp = 5

x_inicial = 0
y_inicial = 0
theta_inicial = 0

x_record = np.zeros(len(time))
y_record = np.zeros(len(time))
theta_record = np.zeros(len(time))
error_record = np.zeros(len(time))
vel_record = np.zeros(len(time))
steering_record = np.zeros(len(time))

x_record[0] = x_inicial
y_record[0] = y_inicial
theta_record[0] = theta_inicial

for i in range(len(time)-1):
    error = mt.sqrt((x_interpolated[i] - x_record[i])**2 + (y_interpolated[i]-y_record[i])**2)-distancia
    velocity = (Kvp * error) + Kip*(error_record[i] + error * dt)
    theta_goal = mt.atan2((y_interpolated[i]-y_record[i]),(x_interpolated[i]-x_record[i]))
    steering_angle = Ktp * (theta_goal - theta_record[i])

    x_new = x_record[i] + velocity * np.cos(theta_record[i]) *dt
    y_new = y_record[i] + velocity * np.sin(theta_record[i]) *dt
    theta_new = theta_record[i] + (velocity/length_wheel) * np.tan(steering_angle*dt)

    x_record[i+1] = x_new
    y_record[i+1] = y_new
    theta_record[i+1] = theta_new
    error_record[i] = error
    vel_record[i] = velocity
    steering_record[i] = steering_angle

plt.plot(x_record, y_record, color='b', label='Mobile Robot')
plt.plot(x_interpolated, y_interpolated, color='g', label='Planned Trajectory')
plt.scatter(x_points, y_points, color='red', label="Via Points")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.legend()
plt.grid(True)

plt.show()