import pybullet as p
p.connect(p.GUI)
# p.loadURDF("urdf/legged_robot/legged_robot.urdf")
p.loadURDF("urdf/simbicon_urdf/biped2d.urdf")
while (1):
  p.stepSimulation()
