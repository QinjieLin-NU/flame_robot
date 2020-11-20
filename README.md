# Reninforment Learning with Bipedal

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