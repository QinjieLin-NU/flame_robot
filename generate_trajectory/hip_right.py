import numpy as np

p0 = [0, 0]

p1 = [2, np.radians(-20)]
p2 = [4, np.radians(-40)]
p3 = [5, np.radians(-40)]
p4 = [6, np.radians(-45)]
p5 = [9, 0]
p6 = [10, np.radians(-20)]

p7 = [11, np.radians(-20)] # +9
points = [p1,p2,p3,p4,p5,p6]

def func(P0,P1,t):
    theta = (P1[1]-P0[1])/(P1[0]-P0[0])*(t-P0[0]) + P0[1]
    return theta
t_range = np.arange(0,100000)*0.01
theta_list = []
for t in t_range:
    remain = t*100 % 9
    if remain>=0 and remain<2:
        remain = remain + 9
    if t >= p0[0] and t < p1[0]:
        theta = func(p0,p1,t)
        theta_list.append(theta)
    elif t >= p1[0] and t < p2[0]:
        theta = func(p1,p2,t)
        theta_list.append(theta)
    elif t >= p2[0] and t < p3[0]:
        theta = func(p2,p3,t)
        theta_list.append(theta)
    elif t >= p3[0] and t < p4[0]:
        theta = func(p3,p4,t)
        theta_list.append(theta)
    elif t >= p4[0] and t < p5[0]:
        theta = func(p4,p5,t)
        theta_list.append(theta)
    elif t >= p5[0] and t < p6[0]:
        theta = func(p5,p6,t)
        theta_list.append(theta)
    elif t >= p6[0] and t < p7[0]:
        theta = func(p6,p7,t)
        theta_list.append(theta)
    elif remain >= p1[0] and remain < p2[0]:
        theta = func(p1,p2,t)
        theta_list.append(theta)
    elif remain >= p2[0] and remain < p3[0]:
        theta = func(p2,p3,t)
        theta_list.append(theta)
    elif remain >= p3[0] and remain < p4[0]:
        theta = func(p3,p4,t)
        theta_list.append(theta)
    elif remain >= p4[0] and remain < p5[0]:
        theta = func(p4,p5,t)
        theta_list.append(theta)
    elif remain >= p5[0] and remain < p6[0]:
        theta = func(p5,p6,t)
        theta_list.append(theta)
    elif remain >= p6[0] and remain < p7[0]:
        theta = func(p6,p7,t)
        theta_list.append(theta)

print(theta_list)
np.savetxt('./hip_left.csv',theta_list, delimiter=',')