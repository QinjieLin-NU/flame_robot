import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet
from pybullet_utils import bullet_client

from pkg_resources import parse_version


class BulletBaseEnv(gym.Env):
	"""
	Base gym class for  simulation environments
	"""

	def __init__(self, robot, render=False):
		self.scene = None
		self.physicsClientId = -1
		self.ownsPhysicsClient = 0
		self.camera = Camera()
		self.isRender = render
		self.robot = robot
		self._seed()
		self._cam_dist = 3
		self._cam_yaw = 0
		self._cam_pitch = -30
		self._render_width = 320
		self._render_height = 240
		self.real_time=False


	def configure(self, args):
		self.robot.args = args

	def _seed(self, seed=None):
		self.np_random, seed = gym.utils.seeding.np_random(seed)
		self.robot.np_random = self.np_random  # use the same np_randomizer for robot as for env
		return [seed]

	def _reset(self):
		return 

	def _render(self, mode="realtime", close=False):
		"""
		render_flag: if True then render, if False then close GUi
		mode: if realtime, then add sleep in  step function
		This only supports render on Bullet Engine
		isRender parameter will be passed in Reset Funstion ofE BipedalBaseEnv
		"""
		self.isRender = True
		if(mode == "realtime"):
			self.real_time=True
		else:
			self.real_time=False
		return 

	def _close(self):
		return

	def HUD(self, state, a, done):
		pass

	if parse_version(gym.__version__)>=parse_version('0.9.6'):
		close = _close
		render = _render
		reset = _reset
		seed = _seed

	def camera_adjust(self):
		"""
		This will adjust the camera of pybullt
		TODO:bosy xyz not yet set
		"""
		x, y, z = self.body_xyz
		self.camera_x = 0.98*self.camera_x + (1-0.98)*x
		self.camera.move_and_look_at(self.camera_x, y-2.0, 1.4, x, y, 1.0)


class Camera:
	def __init__(self):
		pass

	def move_and_look_at(self, i, j, k, x, y, z):
		lookat = [x, y, z]
		distance = 10
		yaw = 10
		self._p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)
