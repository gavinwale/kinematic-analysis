# Tested program against forward kinematics solver and it WORKS!
# I get the same answer back that I put in.
import numpy as np
np.set_printoptions(suppress=True)

class NewtonRaphson:
    """A class that implements Newton-Raphson iteration for solving f(x) = 0."""

    def __init__(self, gamma=0.95, guess=np.zeros([1,0], dtype=np.float64)):
        """Initialize the attributes of the NR iteration."""
        self.abstol = 1e-7
        self.reltol = 1e-8
        self.iter = 1
        self.maxiter = 100
        self.gamma = gamma
        self.x0 = guess
        self.xstar = guess
        self.deltax = np.zeros([2,1], dtype=np.float64)
        self.f = np.zeros([2,1], dtype=np.float64)
        self.J = np.zeros((2,2), dtype=np.float64)

    def loop_eqns(self, x):
        """Implement the equations to be solved."""
        theta_dot1, theta_dot2 = x[:,0]

        self.f[0][0] = -theta_dot1*np.sin(theta1)-np.sin(theta1+theta2)*(theta_dot1*theta_dot2)-0.5*np.sin(theta1+theta2+theta3)*(theta_dot1*theta_dot2*theta_dot3) - x_dot
        self.f[1][0] = theta_dot1*np.cos(theta1)+np.cos(theta1+theta2)*(theta_dot1*theta_dot2)+0.5*np.cos(theta1+theta2+theta3)*(theta_dot1*theta_dot2*theta_dot3) - y_dot

    def d_loop_eqns(self, x):
        """Implement the Jacobian of the equations to be solved."""
        theta_dot1, theta_dot2 = x[:,0]

        self.J[0,0] = -np.sin(theta1)-np.sin(theta1+theta2)-0.5*np.sin(theta1+theta2+theta3)
        self.J[0,1] = -np.sin(theta1+theta2)-0.5*np.sin(theta1+theta2+theta3)
        self.J[1,0] = np.cos(theta1)+np.cos(theta1+theta2)+0.5*np.cos(theta1+theta2+theta3)
        self.J[1,1] = np.cos(theta1+theta2)+0.5*np.cos(theta1+theta2+theta3)

    def step(self, x):
        """Take a Newton-Raphson step and decide exit condition."""
        self.loop_eqns(x)
        self.d_loop_eqns(x)
        self.deltax = -self.gamma*( np.linalg.solve(self.J, self.f) )
        self.xstar += self.deltax
        self.iter += 1

        C1 = np.linalg.norm(self.f) > self.abstol
        C2 = np.linalg.norm(self.deltax) > self.reltol
        C3 = self.maxiter > self.iter
        return C1 and C2 and C3


    def solve(self):
        """Keep taking steps until convergence."""
        while self.step(self.xstar): pass

guess = np.array([[-0.001], [-0.002]], dtype=np.float64)
# nr = NewtonRaphson()
theta1 = 1.376319
theta2 = 0.645084
theta3 = 0.621556
theta_dot3 = -0.00366
x_dot = 0.015267
y_dot = 0.008502
nr = NewtonRaphson(guess=guess.copy())
nr.solve()
print(f"Solution:\n {nr.xstar.flatten()}\n\n Number of iterations: {nr.iter}")
