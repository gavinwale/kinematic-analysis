import numpy as np

theta1 = 1.376319
theta2 = 0.645084
theta3 = 0.621556
theta_dot1 = -0.00480623
theta_dot2 = -1.09815809
theta_dot3 = -0.00366

link1 = np.array([[-theta_dot1*np.sin(theta1)],
                  [theta_dot1*np.cos(theta1)]])
                  
link2 = np.array([[-theta_dot2*np.sin(theta1+theta2)-theta_dot1*np.sin(theta1+theta2)],
                  [theta_dot2*np.cos(theta1+theta2)+theta_dot1*np.cos(theta1+theta2)]])
                  
link3 = np.array([[0.5*(-theta_dot3*np.sin(theta1+theta2+theta3)-theta_dot2*np.sin(theta1+theta2+theta3)-theta_dot1*np.sin(theta1+theta2+theta3))],
                  [0.5*(theta_dot3*np.cos(theta1+theta2+theta3)+theta_dot2*np.cos(theta1+theta2+theta3)+theta_dot1*np.cos(theta1+theta2+theta3))]])
                  
print(link1+link2+link3)
 