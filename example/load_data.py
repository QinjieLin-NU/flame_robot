import pickle

def get_thetas(file_name):
    """
    return a theta array[1000,7]: theta of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    """
    data_dict = None
    with open(file_name, 'rb') as f:
        data_dict = pickle.load(f)
    state = data_dict["state_list"] #including vel, theta and collsision
    motor_thetas = state[:,8:22:2]
    return motor_thetas

def get_torques(file_name):
    """
    return a torque array[1000,7]: torque of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    """
    data_dict = None
    with open(file_name, 'rb') as f:
        data_dict = pickle.load(f)
    action = data_dict["action_list"] #toque of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    return action

theta_list = get_thetas(file_name='example/data/exp4_trajectory.pkl')
toruqe_list = get_torques(file_name='example/data/exp4_trajectory.pkl')
print(theta_list.shape,toruqe_list.shape)
print(theta_list)