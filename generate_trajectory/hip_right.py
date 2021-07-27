import numpy as np
import matplotlib.pyplot as plt

def cal_k(P0,P1):
    k = (P1[1]-P0[1])/(P1[0]-P0[0])
    return k
p0 = [0, 0]

p1 = [2, np.radians(20)]
p2 = [5, np.radians(-30)]
k1 = cal_k(p1,p2)
p3 = [6, np.radians(-10)]
k2 = cal_k(p2,p3)
p4 = [7, np.radians(-10)]
k3 = cal_k(p3,p4)
p5 = [10, np.radians(10)]
k4 = cal_k(p4,p5)
p6 = [11, np.radians(10)]
k5 = cal_k(p5,p6)

p7 = [12, np.radians(20)] # +9
k6 = cal_k(p6,p7)

def func(P0,P1,t):
    theta = (P1[1]-P0[1])/(P1[0]-P0[0])*(t-P0[0]) + P0[1]
    return theta
def func2(P0,k,t):
    y = k*(t-P0[0])+P0[1]
    return y

t_range = np.arange(0,100000)*0.01
theta_list = []
for t in t_range:
    if t > 2:
        remain = t % 10
        if remain>=0 and remain<2:
            remain = remain + 10

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
        p1_x = p1[0] + t // 10 * 10
        p1_copy = [p1_x,p1[1]]
        theta = func2(p1_copy,k1,t)
        theta_list.append(theta)
    elif remain >= p2[0] and remain < p3[0]:
        p2_x = p2[0] + t // 10 * 10
        p2_copy = [p2_x,p2[1]]
        theta = func2(p2_copy,k2,t)
        theta_list.append(theta)
    elif remain >= p3[0] and remain < p4[0]:
        p3_x = p3[0] + t // 10 * 10
        p3_copy = [p3_x,p3[1]]
        theta = func2(p3_copy,k3,t)
        theta_list.append(theta)
    elif remain >= p4[0] and remain < p5[0]:
        p4_x = p4[0] + t // 10 * 10
        p4_copy = [p4_x,p4[1]]
        theta = func2(p4_copy,k4,t)
        theta_list.append(theta)
    elif remain >= p5[0] and remain < p6[0]:
        p5_x = p5[0]%10 + t // 10 * 10
        p5_copy = [p5_x,p5[1]]
        theta = func2(p5_copy,k5,t)
        theta_list.append(theta)
    elif remain >= p6[0] and remain < p7[0]:
        p6_x = p6[0]%10 + t // 10 * 10
        p6_copy = [p6_x,p6[1]]
        theta = func2(p6_copy,k6,t)
        theta_list.append(theta)

print(theta_list)
np.savetxt('./hip_right.csv',theta_list, delimiter=',')
xpoint = t_range
plt.plot(t_range, theta_list, label = 'x')
plt.show()