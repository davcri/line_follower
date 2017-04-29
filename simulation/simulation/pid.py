import math

'''
calculatePID globals
'''
previousError = 0
kp = 1
ki = 1
kd = 1
i = 0
d = 0
p = 0


def calculatePID(currentValue, target, dt):
    global p, i, d, previousError
    error = target - currentValue

    p = error
    i = i + error*dt
    d = (error-previousError)/dt
    if math.fabs(d) > 20:
        d = 0

    previousError = error
    # time.sleep(dt)

    return kp*p + ki*i + kd*d
