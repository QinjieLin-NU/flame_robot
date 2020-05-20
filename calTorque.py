import numpy as np

#read txt file function
def read_txt(filename):
    f = open(filename, "r")
    l = []
    l = [ line.split() for line in f]

    weights_string = np.array(l)
    weights = weights_string.astype(np.float)
    return weights

#n is the index of joint
def networkOutput(robot,weight,offset,scale,n):
    lw = weight[:,n]
    lioff = offset[:,n]
    liscale = scale[:,n]

    output = 0
    output += (robot.center_hip.q               - lioff[0])  * liscale[0]  * lw[0]
    output += (robot.center_hip.qd              - lioff[1])  * liscale[1]  * lw[1]
    output += (robot.left_hip.q           - lioff[2])  * liscale[2]  * lw[2]
    output += (robot.left_hip.qd          - lioff[3])  * liscale[3]  * lw[3]
    output += (robot.right_hip.q           - lioff[4])  * liscale[4]  * lw[4]
    output += (robot.right_hip.qd          - lioff[5])  * liscale[5]  * lw[5]
    output += (robot.left_knee.q           - lioff[6])  * liscale[6]  * lw[6]
    output += (robot.left_knee.qd          - lioff[7])  * liscale[7]  * lw[7]
    output += (robot.right_knee.q           - lioff[8])  * liscale[8]  * lw[8]
    output += (robot.right_knee.qd          - lioff[9])  * liscale[9]  * lw[9]
    output += (robot.left_ankleY.q         - lioff[10]) * liscale[10] * lw[10]
    output += (robot.left_ankleY.qd        - lioff[11]) * liscale[11] * lw[11]
    output += (robot.right_ankleY.q         - lioff[12]) * liscale[12] * lw[12]
    output += (robot.right_ankleY.qd        - lioff[13]) * liscale[13] * lw[13]

    # output += (s.l().anklex.q         - lioff[14]) * liscale[14] * lw[14]
    # output += (s.l().anklex.qd        - lioff[15]) * liscale[15] * lw[15]
    # output += (s.r().anklex.q         - lioff[16]) * liscale[16] * lw[16]
    # output += (s.r().anklex.qd        - lioff[17]) * liscale[17] * lw[17]
    # output += (s.l().foot.back.state  - lioff[18]) * liscale[18] * lw[18]
    # output += (s.l().foot.front.state - lioff[19]) * liscale[19] * lw[19]
    # output += (s.r().foot.back.state  - lioff[20]) * liscale[20] * lw[20]
    # output += (s.r().foot.front.state - lioff[21]) * liscale[21] * lw[21]

    output += (robot.torso.yaw              - lioff[22]) * liscale[22] * lw[22]
    output += (robot.torso.yawd             - lioff[23]) * liscale[23] * lw[23]
    output += (robot.torso.pitch            - lioff[24]) * liscale[24] * lw[24]
    output += (robot.torso.pitchd           - lioff[25]) * liscale[25] * lw[25]
    output += (robot.torso.roll             - lioff[26]) * liscale[26] * lw[26]
    output += (robot.torso.rolld            - lioff[27]) * liscale[27] * lw[27]
    output += (robot.bias                 - lioff[28]) * liscale[28] * lw[28]

    return output



def cal_Torque(robot):
    weight = read_txt("standweight.txt")
    offset = read_txt("standoffset.txt")
    scale = read_txt("standscale.txt")

    #get applied_torques
    applied_torques = []
    for i in range(7):
        torque_i = networkOutput(robot,weight,offset,scale,i)
        applied_torques.append(torque_i)

    return applied_torques
