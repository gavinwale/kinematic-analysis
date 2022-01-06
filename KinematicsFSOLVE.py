import scipy.optimize as opt
import numpy as np

# Numerical Solution for Problem 4
def f(variables):
    (theta2,theta3) = variables
    eq1 = 50*np.cos(np.pi/4) + 175*np.cos(theta2) - 100*np.cos(theta3) - 150
    eq2 = 50*np.sin(np.pi/4) + 175*np.sin(theta2) - 100*np.sin(theta3)

    return [eq1,eq2]

solution = opt.fsolve(f, (0.1,1))
print(solution)

theta2 = solution[0]
theta3 = solution[1]

def f(variables):
    (thetaT,dE) = variables
    eq3 = 150*np.cos(theta3) + 150*np.cos(thetaT) - dE
    eq4 = 150*np.sin(theta3) - 150*np.sin(thetaT) + 50

    return [eq3,eq4]

solution = opt.fsolve(f, (0.1,1))
print(solution)
