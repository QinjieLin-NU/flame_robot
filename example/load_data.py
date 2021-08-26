import pickle

def get_thetas():
    """
    return a theta array[1000,7]: theta of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    """
    data_dict = None
    with open('example/data/exp2_trajectory.pkl', 'rb') as f:
        data_dict = pickle.load(f)
    state = data_dict["state_list"] #including vel, theta and collsision
    motor_thetas = state[:,8:22:2]
    return motor_thetas

def get_torques():
    """
    return a torque array[1000,7]: torque of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    """
    data_dict = None
    with open('example/data/exp2_trajectory.pkl', 'rb') as f:
        data_dict = pickle.load(f)
    action = data_dict["action_list"] #toque of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    return action

theta_list = get_thetas()
toruqe_list = get_torques()
print(theta_list.shape,toruqe_list.shape)
print(theta_list)