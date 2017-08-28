# Simple python program to simulate the PID controller response
# with steering values supplied by the C++ pid program
# (with or without twiddle)

import numpy as np
import matplotlib.pyplot as plt

#Kp, Ki, Kd = 0.01, 0.0, 0.0
#Kp, Ki, Kd = 0.05, 0.0, 0.0
#Kp, Ki, Kd = 0.1, 0.0, 0.0
#Kp, Ki, Kd = 0.5, 0.0, 0.0
#Kp, Ki, Kd = 0.05, 0.0001, 0.0
#Kp, Ki, Kd = 0.05, 0.0005, 0.0
#Kp, Ki, Kd = 0.05, 0.001, 0.0
#Kp, Ki, Kd = 0.05, 0.001, 0.5
#Kp, Ki, Kd = 0.05, 0.001, 1.0
#Kp, Ki, Kd = 0.05, 0.001, 2.0

#Kp, Ki, Kd = 0.06, 0.0012, 0.75

Kp, Ki, Kd = 0.250829, 0.00256535, 5.91271

cte_arr = []
n = 2800 #1900 #900 #620 #175 #225 #380 #190 #390 #400 #250

with open('../build/result.txt') as f:
    for i in range(2*n):
        cte_val = float(f.readline().strip())
        cte_arr.append(cte_val)

plt.plot(range(2*n), cte_arr, 'g', label='PID controller response')
plt.plot(range(2*n), np.zeros(2*n), 'r', label='reference')
plt.grid(True)
plt.title('PID Controller (Kp={}, Ki={}, Kd={})'.format(Kp,Ki,Kd))
plt.xlabel('Iterations')
plt.ylabel('Steering')
plt.show()
