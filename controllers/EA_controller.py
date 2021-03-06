import numpy as np
import copy
class EA_weights_Controller():
#read txt file function
    def __init__(self,robot):
        #read weights
        self.weight = self.read_csv("controllers/walkweight.csv")
        self.offset = self.read_csv("controllers/walkoffset.csv")
        self.scale = self.read_csv("controllers/walkscale.csv")
        self.kp=self.weight[26,-7:]
        self.kd=self.weight[27,-7:]

        #init refPrev
        self.center_hip_q_refPrev = 0
        self.upperbody_q_refPrev = 0
        self.interleg_q_refPrev = 0
        self.stance_knee_q_refPrev = 0
        self.swing_knee_q_refPrev = 0
        self.stance_ankle_q_refPrev = 0
        self.swing_ankle_q_refPrev = 0

        #init robot
        self.robot = robot

    def read_txt(self,filename):
        f = open(filename, "r")
        l = []
        l = [ line.split() for line in f]

        # print("l",l)
        weights = np.array([np.array(xi) for xi in l])

        # weights_string = np.array(l)
        # print("weights_string",weights_string)
        # weights = weights_string.astype(np.float)
        # weights = np.array(l)
        # print("weights",weights)
        # weights = weights.astype(np.float)
        return weights

    def read_csv(self,filename):
        # f = open(filename, "r")
        data = np.loadtxt(filename, delimiter=',')
        weights = data
        
        weights = np.array(weights)
        # print("weights",weights)
        return weights

        

    #n is the index of joint
    def networkOutput(self,robot,weight,offset,scale,n):
        #load weight
        lw = weight[:,n]
        lioff = offset[:,n]
        liscale = scale[:,n]

        #judge which leg is stance
        if robot.left_foot.state == 1:
            left_is_stance = True
        else:
            left_is_stance = False

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

        output += (robot.__dict__[stance_leg+'_knee'].q   - lioff[6])  * liscale[6]  * lw[6]
        output += (robot.__dict__[swing_leg+'_knee'].q    - lioff[7])  * liscale[7]  * lw[7]
        output += (robot.__dict__[stance_leg+'_ankleY'].q - lioff[8])  * liscale[8]  * lw[8]
        output += (robot.__dict__[swing_leg+'_ankleY'].q  - lioff[9])  * liscale[9]  * lw[9]
        output += (robot.__dict__[stance_leg+'_ankleX'].q - lioff[10]) * liscale[10] * lw[10]
        output += (robot.__dict__[swing_leg+'_ankleX'].q  - lioff[11]) * liscale[11] * lw[11]


        #Derivatives
        output += (adj_flag * robot.torso.rolld              - lioff[12]) * liscale[12] * lw[12]
        output += (robot.torso.pitchd        - lioff[13]) * liscale[13] * lw[13]
        output += (adj_flag * robot.torso.yawd               - lioff[14]) * liscale[14] * lw[14]
        output += (robot.center_hip.qd           - lioff[15]) * liscale[15] * lw[15]

        st_thighd = robot.__dict__[stance_leg+'_hip'].qd - robot.torso.pitchd
        output += (st_thighd              - lioff[16]) * liscale[16] * lw[16]
        
        interlegd = robot.__dict__[swing_leg+'_hip'].qd  - robot.__dict__[stance_leg+'_hip'].qd
        output += (interlegd              - lioff[17]) * liscale[17] * lw[17]

        output += (robot.__dict__[stance_leg+'_knee'].qd  - lioff[18]) * liscale[18] * lw[18]
        output += (robot.__dict__[swing_leg+'_knee'].qd   - lioff[19]) * liscale[19] * lw[19]
        output += (robot.__dict__[stance_leg+'_ankleY'].qd- lioff[20]) * liscale[20] * lw[20]
        output += (robot.__dict__[swing_leg+'_ankleY'].qd - lioff[21]) * liscale[21] * lw[21]
        output += (robot.__dict__[stance_leg+'_ankleX'].qd- lioff[22]) * liscale[22] * lw[22]
        output += (robot.__dict__[swing_leg+'_ankleX'].qd - lioff[23]) * liscale[23] * lw[23]


        doublestance = 0
        output += (doublestance           - lioff[24]) * liscale[24] * lw[24]
        output += (robot.bias              - lioff[25]) * liscale[25] * lw[25]   
        
        # output = output/360 * np.pi
        # output = 1.0/(1.0+np.exp(-(output)))
        output = np.arctan2(np.tanh(output),1)
        return output


    #set the calculated ref and p, d gains
    def assignRef(self):
        robot = self.robot
        weight = self.weight[0:29,:-7]

        offset = self.offset[0:29,:-7]
        scale = self.scale[0:29,:-7]

        self.center_hip_q_ref = self.networkOutput(robot,weight,offset,scale,0)
        # print("7 output:",self.center_hip_q_ref)
        self.upperbody_q_ref = self.networkOutput(robot,weight,offset,scale,1)
        self.interleg_q_ref = self.networkOutput(robot,weight,offset,scale,2)
        self.stance_knee_q_ref = self.networkOutput(robot,weight,offset,scale,3)
        self.swing_knee_q_ref = self.networkOutput(robot,weight,offset,scale,4)
        self.stance_ankle_q_ref = self.networkOutput(robot,weight,offset,scale,5)
        self.swing_ankle_q_ref = self.networkOutput(robot,weight,offset,scale,6)

        # print("weight",weight)
        # print("shape",weight.shape)


        #calculate qd, dt=1
        dt = 0.01
        self.center_hip_qd_ref = self.FiltandDerivRef(self.center_hip_q_ref,self.center_hip_q_refPrev, dt)
        self.upperbody_qd_ref = self.FiltandDerivRef(self.upperbody_q_ref,self.upperbody_q_refPrev, dt)
        self.interleg_qd_ref = self.FiltandDerivRef(self.interleg_q_ref,self.interleg_q_refPrev, dt)
        self.stance_knee_qd_ref = self.FiltandDerivRef(self.stance_knee_q_ref,self.stance_knee_q_refPrev, dt)
        self.swing_knee_qd_ref = self.FiltandDerivRef(self.swing_knee_q_ref,self.swing_knee_q_refPrev, dt)
        self.stance_ankle_qd_ref = self.FiltandDerivRef(self.stance_ankle_q_ref,self.stance_ankle_q_refPrev, dt)
        self.swing_ankle_qd_ref = self.FiltandDerivRef(self.swing_ankle_q_ref,self.swing_ankle_q_refPrev, dt)

        return 

    def FiltandDerivRef(self,ref,refPrev, dt):
        alpha = 1.0
        ref = alpha * ref +  (1.0-alpha)*refPrev
        qd_ref = (ref - refPrev)/dt
        refPrev = copy.deepcopy(ref)
        return qd_ref


    # calTau
    def update(self):
        robot = self.robot
        kp = self.kp
        kd = self.kd
        #assign references
        self.assignRef()

        #define centerhipx, upperbody, interleg, stance_knee, swing knee, stance ankle, swing ankle
        if robot.left_foot.state == 1:
            left_is_stance = True
        else:
            left_is_stance = False


        if left_is_stance:
            stance_leg = 'left'
            swing_leg = 'right'
            adj_flag = -1.0
        else:
            stance_leg = 'right'
            swing_leg = 'left'
            adj_flag = 1.0

        center_hip_q = robot.center_hip.q
        center_hip_qd = robot.center_hip.qd
        interleg_q  = robot.__dict__[swing_leg+'_hip'].q   - robot.__dict__[stance_leg+'_hip'].q
        interleg_qd = robot.__dict__[swing_leg+'_hip'].qd  - robot.__dict__[stance_leg+'_hip'].qd

        center_hip_tau = kp[0]*(self.center_hip_q_ref-center_hip_q)+kd[0]*(self.center_hip_qd_ref-center_hip_qd)
        upperbody_tau = kp[1]*(self.upperbody_q_ref-robot.torso.pitch)+kd[1]*(self.upperbody_qd_ref-robot.torso.pitchd)
        interleg_tau = kp[2]*(self.interleg_q_ref-interleg_q)+kd[2]*(self.interleg_qd_ref-interleg_qd)
        stance_knee_tau = kp[3]*(self.stance_knee_q_ref-robot.__dict__[stance_leg+'_knee'].q)+kd[3]*(self.stance_knee_qd_ref-robot.__dict__[stance_leg+'_knee'].qd)
        swing_knee_tau = kp[4]*(self.swing_knee_q_ref-robot.__dict__[swing_leg+'_knee'].q)+kd[4]*(self.swing_knee_qd_ref-robot.__dict__[swing_leg+'_knee'].qd)
        stance_ankle_tau = kp[5]*(self.stance_ankle_q_ref-robot.__dict__[stance_leg+'_ankleY'].q)+kd[5]*(self.stance_ankle_qd_ref-robot.__dict__[stance_leg+'_ankleY'].qd) #ankle y or ankle x or just ankle?
        swing_ankle_tau = kp[6]*(self.swing_ankle_q_ref-robot.__dict__[swing_leg+'_ankleY'].q)+kd[6]*(self.swing_ankle_qd_ref-robot.__dict__[swing_leg+'_ankleY'].qd)

        swing_hipy_tau = 0
        stance_hipy_tau = 0

        swing_hipy_tau += interleg_tau
        stance_hipy_tau -= interleg_tau
        stance_hipy_tau -= upperbody_tau




        #centerHip_torque
        centerHip_torque = center_hip_tau
        #rightHip_torque, leftHip_torque, rightKnee_torque, leftKnee_torque, rightAnkleY_torque, leftAnkleY_torque
        if left_is_stance:
            leftHip_torque = stance_hipy_tau
            rightHip_torque = swing_hipy_tau
            leftKnee_torque = stance_knee_tau
            rightKnee_torque = swing_knee_tau
            leftAnkleY_torque = stance_ankle_tau
            rightAnkleY_torque = swing_ankle_tau
        else:
            leftHip_torque = swing_hipy_tau
            rightHip_torque = stance_hipy_tau
            leftKnee_torque = swing_knee_tau
            rightKnee_torque = stance_knee_tau
            leftAnkleY_torque = swing_ankle_tau
            rightAnkleY_torque = stance_ankle_tau            


        torques = [centerHip_torque,rightHip_torque,rightKnee_torque,rightAnkleY_torque,leftHip_torque,leftKnee_torque,leftAnkleY_torque]
        #torques = [max(min(x, 1), -1) for x in torques]
        return torques
            

class PID_Controller():
#read txt file function
    def __init__(self,robot):

        self.hipytrqCtrlkp = 20
        self.hipytrqCtrlkd = 0.25

        #init refPrev
        self.center_hip_q_refPrev = 0
        self.upperbody_q_refPrev = 0
        self.interleg_q_refPrev = 0
        self.stance_knee_q_refPrev = 0
        self.swing_knee_q_refPrev = 0
        self.stance_ankle_q_refPrev = 0
        self.swing_ankle_q_refPrev = 0

        #init robot
        self.robot = robot    

    def compute_quintic_spline(self, t, x0,  v0,  a0, x1,  v1,  a1):
        t = np.clip(t,0,1)
        t2 = t*t
        t3 = t2*t
        t4 = t3*t
        t5= t4*t
        f = x0
        e = v0
        d = 0.5 * a0

        # compute the rightmost column of the solution for [a b ]
        y1 = x1 -   d - e - f
        y2 = v1 - 2*d - e
        y3 = a1 - 2*d

        # multiply out the solution matrix to find [a b c]
        a =   6*y1 - 3*y2 + 0.5*y3
        b = -15*y1 + 7*y2 -     y3
        c =  10*y1 - 4*y2 + 0.5*y3

        # compute the polynomial
        return ( a*t5 + b*t4 + c*t3 + d*t2 + e*t + f )

    def assignRef(self, t):
        # generate hip (interleg) swing trajectory
        #if t < 0.6:
        if t < 600:
        # compute_quintic_spline( 1.67*(Controller()->TimeElapsed()), 0.6, 0.0, -25.0, -0.6, 0.0, 0.0)
            self.interleg_q_ref = self.compute_quintic_spline( 1.67*(t/600), 0.6, 0.0, -25.0, -0.6, 0.0, 0.0)
        else:
            self.interleg_q_ref = -0.6

        dt = 0.01
        self.interleg_qd_ref = self.FiltandDerivRef(self.interleg_q_ref,self.interleg_q_refPrev, dt)


    def FiltandDerivRef(self,ref,refPrev, dt):
        alpha = 1.0
        ref = alpha * ref +  (1.0-alpha)*refPrev
        qd_ref = (ref - refPrev)/dt
        refPrev = copy.deepcopy(ref)
        return qd_ref


    def update(self,t):
        robot = self.robot
        self.assignRef(t)

        if robot.left_foot.state == 1:
            left_is_stance = True
        else:
            left_is_stance = False

        if left_is_stance:
            stance_leg = 'left'
            swing_leg = 'right'
            adj_flag = -1.0
        else:
            stance_leg = 'right'
            swing_leg = 'left'
            adj_flag = 1.0

        interleg_q  = robot.__dict__[swing_leg+'_hip'].q   - robot.__dict__[stance_leg+'_hip'].q
        interleg_qd = robot.__dict__[swing_leg+'_hip'].qd  - robot.__dict__[stance_leg+'_hip'].qd

        center_hip_tau = 0
        upperbody_tau = 0
        interleg_tau = self.hipytrqCtrlkp*(self.interleg_q_ref-interleg_q)+self.hipytrqCtrlkp*(self.interleg_qd_ref-interleg_qd)
        stance_knee_tau = 0
        swing_knee_tau = 0
        stance_ankle_tau = 0
        swing_ankle_tau = 0


        swing_hipy_tau = 0
        stance_hipy_tau = 0

        swing_hipy_tau += interleg_tau
        stance_hipy_tau -= interleg_tau

        #centerHip_torque
        centerHip_torque = center_hip_tau
        #rightHip_torque, leftHip_torque, rightKnee_torque, leftKnee_torque, rightAnkleY_torque, leftAnkleY_torque
        if left_is_stance:
            leftHip_torque = stance_hipy_tau
            rightHip_torque = swing_hipy_tau
            leftKnee_torque = stance_knee_tau
            rightKnee_torque = swing_knee_tau
            leftAnkleY_torque = stance_ankle_tau
            rightAnkleY_torque = swing_ankle_tau
        else:
            leftHip_torque = swing_hipy_tau
            rightHip_torque = stance_hipy_tau
            leftKnee_torque = swing_knee_tau
            rightKnee_torque = stance_knee_tau
            leftAnkleY_torque = swing_ankle_tau
            rightAnkleY_torque = stance_ankle_tau            


        torques = [centerHip_torque,rightHip_torque,rightKnee_torque,rightAnkleY_torque,leftHip_torque,leftKnee_torque,leftAnkleY_torque]
        #torques = [max(min(x, 1), -1) for x in torques]
        return torques
        
