# Reninforment Learning with Bipedal Robot

## Bipedal envs in Pybullet

```
BipedalBulletEnv
|
└───BipedalBaseEnv: implementaton of step, reset, reward function
│   └───BulletBaseEnv: implementaton of render function
│  
└───BipedalRobot: implementation of 
    └───URDFBaseRobot: implementation of loading urdf, applying torques, reset simulation in Pybullet
```  

## Test
```
conda create --name <env> python=3.7
conda activate <env>
pip install -r requirements.txt
python test.py
```

![image info](./images/bipedal_pybulley.png)