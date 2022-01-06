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

    R12 = np.array([[np.cos(b),-np.sin(b),0],
                    [np.sin(b),np.cos(b),0],
                    [0,0,1]])

    R23 = np.array([[np.cos(c),-np.sin(c),0],
                    [np.sin(c),np.cos(c),0],
                    [0,0,1]])

    R02 = np.matmul(R01,R12)

    R03 = np.matmul(R02,R23)

    # Forward Position Level Eqaution

    xy = np.matmul(R01,link0) + np.matmul(R02,link1) + np.matmul(R03,link2)

    print(xy)
