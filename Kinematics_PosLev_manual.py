import numpy as np
import csv
import itertools

# Forward Kinematics 

# Extract Link Angles
theta1 = 1.54925999
theta2 = 0.24825793
theta3 = np.pi/3

# Frame 1
link0 = np.array([[1],
                  [0],
                  [0]])
# Frame 2
link1 = np.array([[1],
                  [0],
                  [0]])
# Frame 3
link2 = np.array([[0.5],
                  [0],
                  [0]])

# Rotation Matrices
R01 = np.array([[np.cos(theta1),-np.sin(theta1),0],
                [np.sin(theta1),np.cos(theta1),0],
                [0,0,1]])

R12 = np.array([[np.cos(theta2),-np.sin(theta2),0],
                [np.sin(theta2),np.cos(theta2),0],
                [0,0,1]])

R23 = np.array([[np.cos(theta3),-np.sin(theta3),0],
                [np.sin(theta3),np.cos(theta3),0],
                [0,0,1]])

R02 = np.matmul(R01,R12)

R03 = np.matmul(R02,R23)

    # Forward Position Level Eqaution

xy = np.matmul(R01,link0) + np.matmul(R02,link1) + np.matmul(R03,link2)

print(xy)
