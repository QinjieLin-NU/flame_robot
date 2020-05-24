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

    left_is_stance = True
    if left_is_stance:
        stance_leg = 'left'
        swing_leg = 'right'
        adj_flag = -1.0
    else:
        stance_leg = 'right'
        swing_leg = 'left'
        adj_flag = 1.0

        
    output = 0
    output += (adj_flag * robot.torso.roll - lioff[0])  * liscale[0]  * lw[0]
    output += (robot.torso.pitch  - lioff[1]) * liscale[1] * lw[1]
    output += (adj_flag * robot.torso.yaw - lioff[2]) * liscale[2] * lw[2]
    output += (robot.center_hip.q               - lioff[3])  * liscale[3]  * lw[3]

    st_thigh  = robot.__dict__[stance_leg+'_hip'].q  - robot.torso.pitch
    output += (st_thigh               - lioff[4])  * liscale[4]  * lw[4]

    interleg  = robot.__dict__[swing_leg+'_hip'].q   - robot.__dict__[stance_leg+'_hip'].q
    output += (interleg               - lioff[5])  * liscale[5]  * lw[5]

    output += (robot.__dict__[stance_leg+'knee'].q   - lioff[6])  * liscale[6]  * lw[6]
    output += (robot.__dict__[swing_leg+'knee'].q    - lioff[7])  * liscale[7]  * lw[7]
    output += (robot.__dict__[stance_leg+'ankley'].q - lioff[8])  * liscale[8]  * lw[8]
    output += (robot.__dict__[swing_leg+'ankley'].q  - lioff[9])  * liscale[9]  * lw[9]
    output += (robot.__dict__[stance_leg+'anklex'].q - lioff[10]) * liscale[10] * lw[10]
    output += (robot.__dict__[swing_leg+'anklex'].q  - lioff[11]) * liscale[11] * lw[11]


    #Derivatives
    output += (adj_flag * robot.torso.rolld              - lioff[12]) * liscale[12] * lw[12]
    output += (robot.torso.pitchd        - lioff[13]) * liscale[13] * lw[13]
    output += (adj_flag * robot.torso.yawd               - lioff[14]) * liscale[14] * lw[14]
    output += (robot.center_hip.qd           - lioff[15]) * liscale[15] * lw[15]

    st_thighd = robot.__dict__[stance_leg+'_hip'].qd - robot.torso.pitchd
    output += (st_thighd              - lioff[16]) * liscale[16] * lw[16]
    
    interlegd = robot.__dict__[swing_leg+'_hip'].qd  - robot.__dict__[stance_leg+'_hip'].qd
    output += (interlegd              - lioff[17]) * liscale[17] * lw[17]

    output += (robot.__dict__[stance_leg+'knee'].qd  - lioff[18]) * liscale[18] * lw[18]
    output += (robot.__dict__[swing_leg+'knee'].qd   - lioff[19]) * liscale[19] * lw[19]
    output += (robot.__dict__[stance_leg+'ankley'].qd- lioff[20]) * liscale[20] * lw[20]
    output += (robot.__dict__[swing_leg+'ankley'].qd - lioff[21]) * liscale[21] * lw[21]
    output += (robot.__dict__[stance_leg+'anklex'].qd- lioff[22]) * liscale[22] * lw[22]
    output += (robot.__dict__[swing_leg+'anklex'].qd - lioff[23]) * liscale[23] * lw[23]



    output += (doublestance           - lioff[24]) * liscale[24] * lw[24]
    output += (robot.bias              - lioff[25]) * liscale[25] * lw[25]   
    
    return output


#set the calculated ref and p, d gains
def assignRef():
    robot.center_hip.q.ref = networkOutput(robot,weight,offset,scale,1)
    robot.upperbody.q.ref = networkOutput(robot,weight,offset,scale,2)
    robot.interleg.q.ref = networkOutput(robot,weight,offset,scale,3)
    robot.stance_knee.q.ref = networkOutput(robot,weight,offset,scale,4)
    robot.swing_knee.q.ref = networkOutput(robot,weight,offset,scale,5)
    robot.stance_ankle.q.ref = networkOutput(robot,weight,offset,scale,6)
    robot.swing_ankle.q.ref = networkOutput(robot,weight,offset,scale,7)
    kp=weight[26,-7:]
    kd=weight[27,-7:]
    #calculate qd, dt=1
    robot.center_hip.qd.ref =
    robot.upperbody.qd.ref =
    robot.interleg.qd.ref = 
    robot.stance_knee.qd.ref =
    robot.swing_knee.qd.ref =
    robot.stance_ankle.qd.ref =
    robot.swing_ankle.qd.ref =


    return 


# calTau
def update():
    #define interleg, stance_knee, swing knee, stance ankle, swing ankle///what is upperbody???
        left_is_stance = True
    if left_is_stance:
        stance_leg = 'left'
        swing_leg = 'right'
        adj_flag = -1.0
    else:
        stance_leg = 'right'
        swing_leg = 'left'
        adj_flag = 1.0
    robot.interleg.q  = robot.__dict__[swing_leg+'_hip'].q   - robot.__dict__[stance_leg+'_hip'].q
    robot.interleg.qd = robot.__dict__[swing_leg+'_hip'].qd  - robot.__dict__[stance_leg+'_hip'].qd

    robot.center_hip.tau = kp[0]*(robot.center_hip.q.ref-robot.center_hip.q)+kd[0]*(robot.center_hip.qd.ref-robot.center_hip.qd)
    robot.upperbody.tau = kp[1]*(robot.upperbody.q.ref-robot.torso.pitch)+kd[1]*(robot.upperbody.qd.ref-robot.torso.pitchd)
    robot.interleg.tau = kp[2]*(robot.interleg.q.ref-robot.interleg.q)+kd[2]*(robot.interleg.qd.ref-robot.interleg.qd)
    robot.stance_knee.tau = kp[3]*(robot.stance_knee.q.ref-robot.__dict__[stance_leg+'knee'].q)+kd[3]*(robot.stance_knee.qd.ref-robot.__dict__[stance_leg+'knee'].qd)
    robot.swing_knee.tau = kp[4]*(robot.swing_knee.q.ref-robot.__dict__[swing_leg+'knee'].q)+kd[4]*(robot.swing_knee.qd.ref-robot.__dict__[swing_leg+'knee'].qd)
    robot.stance_ankle.tau = kp[5]*(robot.stance_ankle.q.ref-robot.__dict__[stance_leg+'ankley'].q)+kd[5]*(robot.stance_ankle.qd.ref-robot.__dict__[stance_leg+'ankley'].qd) #ankle y or ankle x or just ankle?
    robot.swing_ankle.tau = kp[6]*(robot.swing_ankle.q.ref-robot.__dict__[swing_leg+'ankley'].q)+kd[6]*(robot.swing_ankle.qd.ref-robot.__dict__[swing_leg+'ankley'].qd)



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
