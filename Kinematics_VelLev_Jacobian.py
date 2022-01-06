import numpy as np
import csv
import itertools

theta1 = []
theta2 = []
theta3 = []
theta_dot1 = []
theta_dot2 = []
theta_dot3 = []

with open('fk_question.csv', newline='') as csvfile:
    datareader = csv.reader(csvfile, delimiter=',')
    next(datareader)
    for row in datareader:
        theta1.append(float(row[0]))
        theta2.append(float(row[1]))
        theta3.append(float(row[2]))
        theta_dot1.append(float(row[3]))
        theta_dot2.append(float(row[4]))
        theta_dot3.append(float(row[5]))

for (a,b,c,d,e,f) in zip(theta1,theta2, theta3, theta_dot1, theta_dot2, theta_dot3):
    
    J3 = np.array([[-np.sin(a)-np.sin(a+b)-0.5*np.sin(a+b+c),-np.sin(a+b)-0.5*np.sin(a+b+c),-0.5*np.sin(a+b+c)],
                   [np.cos(a)+np.cos(a+b)+0.5*np.cos(a+b+c),np.cos(a+b)+0.5*np.cos(a+b+c),0.5*np.cos(a+b+c)]])

    ang_vel = np.array([[d],
                        [e],
                        [f]])
                    
    print(np.matmul(J3,ang_vel))

