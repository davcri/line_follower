from simulation.simulation import Simulation
from simulation.pid import calculatePID


max_iterations = 1000
simulation_clock = 100  # hz

sim = Simulation()

pid_values = []
error_values = []

f = open('output/data.txt', 'w')
for i in range(max_iterations):
    error = sim.simulatedError(i/simulation_clock)
    error_values.append(error)

    pidValue = calculatePID(error, 0, 1/simulation_clock)
    pid_values.append(pidValue)

    f.write(str(pidValue) + '\n')

    # https://docs.python.org/3.6/library/string.html#formatstrings
    # console_out = 'pidValue = {0: >12.3f} | dt = {1: >12.3f}'
    # print(console_out.format(pidValue, timer.dt(), sign=' '))

f.close()


import matplotlib.pyplot as plt

plt.plot(pid_values)
plt.plot(error_values)
plt.xlabel('iterations')
plt.ylabel('values')
plt.title('PID values and error')
plt.grid(True)
plt.savefig("output/test.png")
plt.show()
