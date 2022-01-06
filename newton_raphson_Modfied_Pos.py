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
        theta3 = np.pi/3
        xCor = -0.68137690919076
        y = 2.120444377971548
        theta1, theta2 = x[:,0]

        self.f[0][0] = np.cos(theta1) + np.cos(theta1 + theta2) + 0.5*np.cos(theta1 + theta2 + theta3) - xCor
        self.f[1][0] = np.sin(theta1) + np.sin(theta1 + theta2) + 0.5*np.sin(theta1 + theta2 + theta3) - y

    def d_loop_eqns(self, x):
        """Implement the Jacobian of the equations to be solved."""
        theta3 = np.pi/3
        theta1, theta2 = x[:,0]

        self.J[0,0] = -np.sin(theta1) - np.sin(theta1+theta2) - 0.5*np.sin(theta1+theta2+theta3)
        self.J[0,1] = -np.sin(theta1+theta2) - 0.5*np.sin(theta1+theta2+theta3)
        self.J[1,0] = np.cos(theta1) + np.cos(theta1+theta2) + 0.5*np.cos(theta1+theta2+theta3)
        self.J[1,1] = np.cos(theta1+theta2) + 0.5*np.cos(theta1+theta2+theta3)

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

guess = np.array([[1.3763189865602508], [0.6450844101669371]], dtype=np.float64)
# nr = NewtonRaphson()
nr = NewtonRaphson(guess=guess.copy())
nr.solve()
print(f"Solution:\n {nr.xstar.flatten()}\n\n Number of iterations: {nr.iter}")
