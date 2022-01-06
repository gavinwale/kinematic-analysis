import numpy as np
import csv
import itertools

# Forward Kinematics 

# Extract Link Angles
theta1 = []
theta2 = []
theta3 = []

with open('fk_question.csv', newline='') as csvfile:
    datareader = csv.reader(csvfile, delimiter=',')
    next(datareader)
    for row in datareader:
        theta1.append(float(row[0]))
        theta2.append(float(row[1]))
        theta3.append(float(row[2]))

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

for (a, b, c) in zip(theta1,theta2,theta3):
    # Rotation Matrices
    R01 = np.array([[np.cos(a),-np.sin(a),0],
                    [np.sin(a),np.cos(a),0],
                    [0,0,1]])

    R02 = np.array([[np.cos(a+b),-np.sin(a-b),0],
                    [np.sin(a-b),np.cos(a+b),0],
                    [0,0,1]])

    R03_1 = np.cos(c)*np.cos(a+b) + np.sin(a-b)*np.sin(c)
    R03_2 = np.sin(c)*np.cos(a+b) - np.sin(a-b)*np.cos(c)
    R03_3 = np.sin(a-b)*np.cos(c) - np.sin(c)*np.cos(a+b) 
    R03_4 = np.sin(a-b)*np.sin(c) + np.cos(c)*np.cos(a+b)

    R03 = np.array([[R03_1,R03_2,0],
                    [R03_3,R03_4,0],
                    [0,0,1]])

    # Forward Position Level Eqaution

    xy = np.matmul(R01,link0) + np.matmul(R02,link1) + np.matmul(R03,link2)

    print(xy)