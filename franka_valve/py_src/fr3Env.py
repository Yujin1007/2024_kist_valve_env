#!/usr/bin/env python3
import sys
import time

import pandas as pd

sys.path.append('/home/kist-robot2/Franka/weight')
import numpy as np
import sys
from numpy.linalg import inv

import mujoco
import gym
from gym import spaces
from random import random, randint, uniform
from scipy.spatial.transform import Rotation as R
from mujoco import viewer
import tools
import torch

BODY = 1
JOINT = 3
GEOM = 5
MOTION_TIME_CONST = 10.
TASK_SPACE_TIME = 3+1+0.5

RL = 2
MANUAL = 1

RPY = False
XYZRPY = True

JOINT_CONTROL = 1
TASK_CONTROL = 2
HEURISTIC_CIRCULAR_CONTROL = 3
RL_CIRCULAR_CONTROL = 4
RL_CONTROL = 6

def BringClassifier(path):
    classifier = torch.load(path)
    classifier.eval()
    return classifier




class valve_template:

    def __init__(self, RENDERING, TRAIN, OBJ) -> None:
        self.k = 7  # for jacobian calculation
        self.dof = 9  # all joints (include gripper joint)
        self.model_path = "../model/scene_valve.xml"
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        if OBJ == "valve":
            from Handle import controller
            self.controller = controller.CController(self.k)
        elif OBJ == "handle":
            from Valve import controller
            self.controller = controller.CController(self.k)

        self._torque = np.zeros(self.dof, dtype=np.float64)
        self.rendering = True
        self.train = False
        self.env_rand = False
        self.len_hist = 10
        self.downsampling = 100
        self.rendering = RENDERING
        self.train = TRAIN
        self.obj = OBJ
        self.observation_space = self._construct_observation_space()
        self.rotation_action_space, self.force_action_space = self._construct_action_space()
        ## reward weight
        self.reward_range = None


        self.viewer = None
        self.q_range = self.model.jnt_range[:self.k]
        self.qdot_range = np.array([[-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750],
                                    [-2.61, 2.61], [-2.61, 2.61], [-2.61, 2.61]])
        self.tau_range = np.array([[0, 87], [0, 87], [0, 87], [0, 87], [0, 12], [0, 12], [0, 12]])
        self.qdot_init = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q_init = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45), 0, 0, 0, 0]
        # self.q_init = [0, np.deg2rad(-45), 0, np.deg2rad(-135), 0, np.deg2rad(90), np.deg2rad(45), 0, 0, 0, 0]
        self.q_reset = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45), 0.03, 0.03, 0, 0]
        self.episode_number = -1

        self.classifier_clk = BringClassifier("./classifier/model_clk.pt")
        self.classifier_cclk = BringClassifier("./classifier/model_cclk.pt")
        desired_contact_list = ["finger_contact0", "finger_contact1",
                                "finger_contact2", "finger_contact3", "finger_contact4", "finger_contact5",
                                "finger_contact6", "finger_contact7",
                                "finger_contact8", "finger_contact9", "finger0_contact", "finger1_contact",
                                "valve_contact0", "valve_contact1",
                                "handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                "handle_contact4", "handle_contact5", "handle_contact6", "handle_contact7",
                                "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                "handle_contact15", "handle_contact16", "handle_contact18", "handle_contact19",
                                "handle_contact21", "handle_contact22", "handle_contact23"]
        desired_contact_list_finger = ["finger_contact1",
                                       "finger_contact2", "finger_contact3", "finger_contact4",
                                       "finger_contact6", "finger_contact7",
                                       "finger_contact8", "finger_contact9", ]
        desired_contact_list_obj = ["handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                    "handle_contact5", "handle_contact6",
                                    "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                    "handle_contact15", "handle_contact16",
                                    "handle_contact21", "handle_contact22", "handle_contact23", "valve_contact0"]

        self.desired_contact_bid = tools.name2id(self.model, GEOM, desired_contact_list)
        self.desired_contact_finger_bid = tools.name2id(self.model, GEOM, desired_contact_list_finger)
        self.desired_contact_obj_bid = tools.name2id(self.model, GEOM, desired_contact_list_obj)

        self.direction_state = ["clk", "cclk"]
        self.scale = 7
        self.handle_limit = 6
        if self.obj == "handle":
            self.obj_idx = 10

        elif self.obj == "valve":
            self.obj_idx = 9
        # self.obj = "handle"
        # self.obj_idx = 10  # 9 -> valve, 10 -> handle
        self.history_observation = np.zeros([self.len_hist * self.downsampling, self.observation_space.shapes])

    def reset(self, direction=None):
        self.control_mode = 0
        # self.manipulability = []
        self.direction = direction
        # cnt_frame = 0
        env_reset = True
        self.episode_time_elapsed = 0.0
        self.handle_angle = 0.0
        self.torque_data = []
        self.history_observation = np.zeros((self.len_hist * self.downsampling, self.observation_space.shapes))
        self.force = 0.0
        self.force_data = []
        while env_reset:
            self.episode_number += 1

            if self.episode_number % 10 == 0:
                self.scale, self.handle_limit = self.mujoco_xml(self.obj)
                self.model = mujoco.MjModel.from_xml_path(self.model_path)
                self.data = mujoco.MjData(self.model)

            self.start_time = self.data.time + 100
            self.controller.initialize()

            self.data.qpos = self.q_init
            self.data.qvel = self.qdot_init


            r, radius = self.env_randomization(self.scale, self.obj)  # self.obs_object initialize

            self.radius = radius


            self.episode_time = abs(MOTION_TIME_CONST * abs(self.goal_angle - self.init_angle) * radius)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ),
                                 self.data.qpos[self.obj_idx], self.data.qvel[self.obj_idx])
            # self.controller.randomize_env(r, obj, self.scale, self.data.xpos[:22].reshape(66, ), self.init_angle,
            #                                self.goal_angle, MANUAL, RPY)
            self.controller.randomize_env(r, self.obj, self.scale, self.data.xpos[:22].reshape(66, ), self.init_angle,
                                          self.goal_angle, RL, RPY)
            self.controller.control_mujoco()

            self.contact_done = False
            self.bound_done = False
            self.goal_done = False
            self.action_rotation_pre = np.zeros(2)
            self.action_force_pre = np.zeros(1)
            while self.control_mode != RL_CIRCULAR_CONTROL:
                self.control_mode = self.controller.control_mode()

                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ),
                                     self.data.qpos[self.obj_idx], self.data.qvel[self.obj_idx])
                self.controller.control_mujoco()
                obs = self._observation()
                self._torque, _ = self.controller.write()
                for i in range(self.dof - 1):
                    self.data.ctrl[i] = self._torque[i]

                mujoco.mj_step(self.model, self.data)
                self.torque_data.append(self._torque[:7])

                end_effector = self.controller.get_ee()
                done = self._done(end_effector)
                normalized_q = self.obs_q
                if max(abs(normalized_q)) > 0.98:
                    done = True

                if done:
                    break

                if self.rendering:
                    self.render()

            if self.control_mode == RL_CIRCULAR_CONTROL:
                env_reset = False
                self.start_time = self.data.time
                self.q_reset[:self.k] = self.data.qpos[:self.k]

        return obs

    def step(self, action_rotation, action_force):

        done = False
        duration = 0

        action_force = self._rescale_action(action_force)

        while not done:
            ee = self.controller.get_ee()
            done = self._done(ee)
            self.control_mode = self.controller.control_mode()
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ),
                                 self.data.qpos[self.obj_idx], self.data.qvel[self.obj_idx])

            # --- RL controller input ---
            if self.control_mode == RL_CIRCULAR_CONTROL:
                ddrollpitchdot_tmp = action_rotation * self.rotation_action_space.high
                duration += 1
                self.controller.put_action(ddrollpitchdot_tmp, action_force)
            if duration == 10:
                break

            self.controller.control_mujoco()
            self._torque, _ = self.controller.write()
            for i in range(self.dof - 1):
                self.data.ctrl[i] = self._torque[i]
            mujoco.mj_step(self.model, self.data)
            self.torque_data.append(self._torque[:7])
            self.force_data.append(self.controller.get_force())


            if self.rendering:
                self.render()

        ee = self.controller.get_ee()
        obs = self._observation()
        done = self._done(ee)
        reward_rotation, reward_force = self._reward(action_rotation, action_force)
        info = self._info()

        self.action_rotation_pre = action_rotation
        self.action_force_pre = action_force
        return obs, reward_rotation, reward_force, done, info

    def _observation(self):

        q_unscaled = self.data.qpos[0:self.k]
        self.obs_q = (q_unscaled - self.q_range[:, 0]) / (self.q_range[:, 1] - self.q_range[:, 0]) * (1 - (-1)) - 1

        self.obs_ee = self.controller.relative_T_hand()
        tau_unscaled = self.data.ctrl[0:self.k]
        self.obs_tau = (tau_unscaled - self.tau_range[:, 0]) / (self.tau_range[:, 1] - self.tau_range[:, 0]) * (
                    1 - (-1)) - 1

        self.obs_omega = np.array([self.data.qvel[self.obj_idx]])

        obs = np.concatenate((self.obs_q, self.obs_ee, self.obs_tau, self.obs_omega), axis=0)
        self.history_observation[1:] = self.history_observation[:-1]
        self.history_observation[0] = obs
        observation = self.generate_downsampled_observation(self.history_observation)
        return observation

    def _reward(self, action_rotation, action_force):
        raise NotImplementedError("Subclass must implement abstract method")

    def _done(self, end_effector):
        raise NotImplementedError("Subclass must implement abstract method")

    def _info(self):
        info = {
            "collision": self.contact_done,
            "bound": self.bound_done,
        }
        return info
    def _rescale_action(self, action_force):
        raise NotImplementedError("Subclass must implement abstract method")
    def _construct_action_space(self):
        action_space = 2
        action_low = -10 * np.ones(action_space)
        action_high = 10 * np.ones(action_space)
        rotation_action_space = gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

        action_space = 1
        action_low = -1 * np.ones(action_space)
        action_high = 1 * np.ones(action_space)
        force_action_space = gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

        return rotation_action_space, force_action_space

    def _construct_observation_space(self):

        s = {
            'q': spaces.Box(shape=(self.k, 1), low=-1, high=1, dtype=np.float32),
            'relative_matrix': spaces.Box(shape=(16, 1), low=-np.inf, high=np.inf, dtype=np.float_),
            'tau': spaces.Box(shape=(self.k, 1), low=-1, high=1, dtype=np.float_),
            'omega': spaces.Box(shape=(1, 1), low=-np.inf, high=np.inf, dtype=np.float_),
        }

        observation = spaces.Dict(s)
        feature_shape = 0
        for _, v in s.items():
            feature_shape += v.shape[0] * v.shape[1]
        observation.shapes = feature_shape
        return observation

    def generate_downsampled_observation(self, observation_history):
        input_state = np.empty((self.len_hist, self.observation_space.shapes))
        j = 0
        for i in range(0, len(observation_history), self.downsampling):
            input_state[j] = observation_history[i]
            j = j + 1
        return input_state

    def render(self):
        if self.viewer is None:
            self.viewer = viewer.launch_passive(model=self.model, data=self.data)
        else:
            self.viewer.sync()

    def env_randomization(self, scale, obj):
        radius_list = [0.017 * scale, 0.1 / 7 * scale]
        o_margin_list = [[[0], [0.0213 * scale], [0]], [[0], [0], [-0.017 / 7 * scale]]]

        handle_quat_candidate = [[0.25192415, -0.64412663, 0.57897236, 0.4317709],
                                 [-0.49077636, 0.42062713, -0.75930974, 0.07523369],
                                 [0.474576307745582, -0.089013785474907, 0.275616460318178, 0.831197594392378],
                                 [0., -0.707, 0.707, 0.],
                                 [0., -0.707, 0.707, 0.],
                                 [0., -0.707, 0.707, 0.],
                                 [0., -0.707, 0.707, 0.],
                                 [0., -0.707, 0.707, 0.],
                                 [-0.46086475, -0.63305975, 0.39180338, 0.48304156],
                                 [-0.07865809, -0.89033475, 0.16254433, -0.41796684],
                                 [0.70738827, 0., 0., 0.70682518],
                                 [0.70710678, 0.70710678, 0., 0.],
                                 [0.70710678, 0.70710678, 0., 0.],
                                 [0.70710678, 0.70710678, 0., 0.],
                                 [0.70710678, 0.70710678, 0., 0.],
                                 [0.70710678, 0.70710678, 0., 0.]]

        handle_pos_candidate = [[0.52, 0, 0.8],
                                [0.28, -0.3, 0.8],
                                [0.326, 0.232, 0.559 + 0.35],

                                [0.5, -0.2, 0.75],
                                [0.55, 0.3, 0.75],
                                [0.65, 0., 0.85],
                                [0.65, 0., 0.55],
                                [0.55, 0., 0.75],

                                [0.4, 0.3, 0.5],
                                [0.25, 0.25, 0.9],
                                [0.48, 0, 0.9],
                                [0.4, 0, 0.115],
                                [0.580994101778967, -0.045675755104744684, 0.115 + 0.2],
                                [0.580994101778967, -0.045675755104744684, 0.115],
                                [0.5, -0.2, 0.115 + 0.2],
                                [0.45, +0.2, 0.115 + 0.3]]

        valve_quat_candidate = [
            [0.0, 1.0, 0.0, 0.0],
            [-0.707, 0.707, 0.0, 0.0],
            [0.0, -0.707, 0.707, 0.0],
            [0.0, -0.707, 0.707, 0.0],
            [0.0, 0.707, -0.0, 0.707],
            [-0.707, 0.707, 0.0, 0.0]
        ]

        valve_pos_candidate = [
            [0.38, 0.1, 0.45],
            [0.2, 0.4, 0.6],
            [0.28, -0.1, 0.7],
            [0.38, 0.0, 0.5],
            [0.48, 0.0, 0.55],
            [0.3, 0.3, 0.6]
        ]

        if obj == "handle":
            o = 0
            obj_code = [0, 1]
            nobj = "valve"
            quat_candidate = handle_quat_candidate
            pos_candidate = handle_pos_candidate
            self.T_vv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
            rot_axis = 'y'

        elif obj == "valve":
            o = 1
            obj_code = [1, 0]
            nobj = "handle"
            quat_candidate = valve_quat_candidate
            pos_candidate = valve_pos_candidate
            self.T_vv = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            rot_axis = 'z'

        self.o_margin = o_margin_list[o]
        scaled_vector = [self.o_margin[0][0], self.o_margin[1][0], self.o_margin[2][0]]

        radius = radius_list[o]

        bid = mujoco.mj_name2id(self.model, BODY, obj)
        nbid = mujoco.mj_name2id(self.model, BODY, nobj)

        if self.env_rand:

            axis = ['x', 'y', 'z']
            i = randint(0, len(pos_candidate)-1)
            # add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            # ori_quat = R.from_quat(tools.quat2xyzw(quat_candidate[i]))
            # new_quat = add_quat * ori_quat
            # random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()
            add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5) / 2)
            ori_quat = R.from_quat(tools.quat2xyzw(handle_quat_candidate[i]))
            y_rot = R.from_euler(rot_axis, random() * 360, degrees=True)
            new_quat = ori_quat * add_quat * y_rot
            random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()

            add_pos = [(random() - 0.5) / 5, (random() - 0.5) / 5, (random() - 0.5) / 5]
            random_pos = [x + y for x, y in zip(add_pos, pos_candidate[i])]
            if random_pos[2] < 0.01:
                random_pos[2] = 0.01
            # random_pos = [(random() * 0.4 + 0.3), (random()*0.8 - 0.4), random() * 0.7 + 0.1]
            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            # print("quat:",random_quat, "pos: ",random_pos)
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))


        else:

            i = self.episode_number % len(quat_candidate)
            random_quat = quat_candidate[i]
            random_pos = pos_candidate[i]


            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))


        mujoco.mj_step(self.model, self.data)



        # input_data = random_quat + random_pos + [radius]

        self.valve_position = self.model.body_pos[bid] + r.apply(scaled_vector)
        input_data = random_quat + self.valve_position.tolist() + [radius]
        if obj == "handle":
            classifier = self.classifier_clk
            test_input_data = torch.Tensor(input_data).cuda()
            predictions = classifier(test_input_data)
            angles = [4, 5, 6, 7, 8, 13, 14, 15, 16, 17, 22, 23, 24, 25, 26, 31, 32, 33, 34, 35]
            result = torch.argmax(predictions)
            result = angles[result]
        elif obj == "valve":
            result = 0

        init_angle = 2 * np.pi * result / 36

        obj_rotation6d = tools.orientation_quat_to_6d(self.model.body_quat[bid], "mujoco")

        # valve_position = self.model.body_pos[bid] + r.apply(scaled_vector)

        self.obj_pos = random_pos
        self.obj_rotation = r.as_matrix()
        self.normal_vector = self.obj_rotation @ self.o_margin
        self.obj_normal = [0, 0, 0]
        for idx in range(3):
            self.obj_normal[idx] = self.normal_vector[idx][0] + self.obj_pos[idx]
        self.init_angle = init_angle
        if self.obj == "valve":
            self.direction = "cclk"
            self.goal_angle = init_angle + 3 * np.pi
        elif self.obj == "handle":
            self.direction = "clk"
            self.goal_angle = init_angle - 3 * np.pi
        return r.as_matrix().tolist(), radius

    def save_frame_data(self, ee):
        r = R.from_euler('xyz', ee[1][3:6], degrees=False)
        rpyfromvalve_rot = r.inv() * R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)
        ee_align = R.from_euler('z', 45, degrees=True)
        rpyfromvalve = (ee_align * rpyfromvalve_rot).as_matrix()

        xyzfromvalve_rot = (R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)).as_matrix()
        xyzfromvalve_rot = np.concatenate([xyzfromvalve_rot, [[0, 0, 0]]], axis=0)
        xyzfromvalve_rot = np.concatenate(
            [xyzfromvalve_rot, [[self.obj_normal[0]], [self.obj_normal[1]], [self.obj_normal[2]], [1]]], axis=1)

        xyzfromvalve = inv(xyzfromvalve_rot) @ np.array([[ee[1][0]], [ee[1][1]], [ee[1][2]], [1]])

        if len(self.rpyfromvalve_data) == 0:
            self.rpyfromvalve_data = rpyfromvalve.reshape(1, 3, 3)
            self.xyzfromvalve_data = xyzfromvalve[0:3].reshape(1, 3)
            self.gripper_data = ee[2]
        else:
            self.rpyfromvalve_data = np.concatenate([self.rpyfromvalve_data, [rpyfromvalve]], axis=0)
            self.xyzfromvalve_data = np.concatenate([self.xyzfromvalve_data, [xyzfromvalve[0:3].reshape(3)]], axis=0)
            self.gripper_data = np.concatenate([self.gripper_data, ee[2]], axis=0)

    def read_file(self):
        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dr_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dr = list(map(float, f_list))
        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dp_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dp = list(map(float, f_list))

        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dy_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dy = list(map(float, f_list))

    def mujoco_xml(self, obj):
        if self.rendering:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None

        del self.model
        del self.data
        s = randint(5, 8)
        m = randint(3, 7)
        f = random() * 4 + 1  # 1~5
        self.friction = f
        # obj = "handle"

        if obj == "handle":
            handle_xml = f'''
                <mujocoinclude>
                    <compiler angle="radian" meshdir="meshes/" autolimits="true"/>
                    <size njmax="500" nconmax="100" />
                    <visual>
                        <global offwidth="3024" offheight="1680" />
                        <quality shadowsize="4096" offsamples="8" />
                        <map force="0.1" fogend="5" />
                    </visual>


                    <asset>

                        <mesh name="handle_base" file="objects/handle_base.STL" scale="{s} {s} {s}"/>
                        <mesh name="handle_base0" file="objects/handle_base/handle_base000.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle_base1" file="objects/handle_base/handle_base001.obj" scale="{s} {s} {s}"/>

                        <mesh name="handle" file="objects/handle.STL" scale="{s} {s} {s}"/>


                        <mesh name="handle0" file="objects/handle2/handle000.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle1" file="objects/handle2/handle001.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle2" file="objects/handle2/handle002.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle3" file="objects/handle2/handle003.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle4" file="objects/handle2/handle004.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle5" file="objects/handle2/handle005.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle6" file="objects/handle2/handle006.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle7" file="objects/handle2/handle007.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle8" file="objects/handle2/handle008.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle9" file="objects/handle2/handle009.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle10" file="objects/handle2/handle010.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle11" file="objects/handle2/handle011.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle12" file="objects/handle2/handle012.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle13" file="objects/handle2/handle013.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle14" file="objects/handle2/handle014.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle15" file="objects/handle2/handle015.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle16" file="objects/handle2/handle016.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle17" file="objects/handle2/handle017.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle18" file="objects/handle2/handle018.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle19" file="objects/handle2/handle019.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle20" file="objects/handle2/handle020.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle21" file="objects/handle2/handle021.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle22" file="objects/handle2/handle022.obj" scale="{s} {s} {s}"/>
                        <mesh name="handle23" file="objects/handle2/handle023.obj" scale="{s} {s} {s}"/>
                    </asset>

                    <contact>
                        <exclude name="handle_contact" body1="handle_base" body2="handle_handle"/>
                    </contact>

                </mujocoinclude>
            '''

            # Now you can write the XML content to a file
            with open('../model/assets_handle.xml',
                      'w') as file:
                file.write(handle_xml)
            handle_limit_xml = f'''
                        <mujocoinclude>

                            <body name="base_h" pos="0 0 0">
                                <inertial diaginertia="0 0 0" mass="0" pos="0 0 0"/>

                                <body name="handle_base" pos="0 0 0">
                                    <inertial pos="0 0 0" mass="2.7" diaginertia="0.1 0.1 0.1" />
                                    <geom name = "handle_base" type="mesh" rgba="1 1 1 1" mesh="handle_base" class="visual" />
                                    <geom name = "obj_contact0" type="mesh"  mesh="handle_base0" class="collision" />
                                    <geom type="mesh" mesh="handle_base1" class="collision"/>
                                    <body name="handle_handle" pos="0 0 0" >
                                        <inertial pos="0 0 0" quat="0.365653 0.605347 -0.36522 0.605365" mass="0.1" diaginertia="0.1 0.1 0.1" />
                                        <!-- frictionloss : 벨브의 뻑뻑한 정도 결정 키울수록 돌리기 힘듦 , stiffness : 다시 원래 각도로 돌아가려는성질 : 0으로 세팅 -->
                                        <joint name="handle_joint" pos="0 0 0" axis="0 1 0" frictionloss="{f}" damping="0" limited="false" springref="0" stiffness="0" range="-{m} {m}"/>
                                        <geom name = "handle" type="mesh" rgba="1 0 0 1" mesh="handle" class="visual" friction="1 0.1 0.1"/>


                                        <geom name = "handle_contact9" type="mesh"  mesh="handle9" class="collision"/><!--연결부-->
                                        <geom name = "handle_contact13" type="mesh" mesh="handle13" class="collision" /><!--연결부-->
                                        <geom name = "handle_contact14" type="mesh" mesh="handle14" class="collision"/><!--연결부-->
                                        <geom name = "handle_contact17" type="mesh" mesh="handle17" class="collision"/><!--연결부-->
                                        <geom name = "handle_contact20" type="mesh" mesh="handle20" class="collision"/><!--연결부-->

                                        <geom name = "handle_contact4" type="mesh"  mesh="handle4" class="collision" /> <!--연결부십자가-->
                                        <geom name = "handle_contact7" type="mesh"  mesh="handle7" class="collision"/> <!--연결부십자가-->       
                                        <geom name = "handle_contact18" type="mesh" mesh="handle18" class="collision"/><!--연결부십자가-->
                                        <geom name = "handle_contact19" type="mesh" mesh="handle19" class="collision"/><!--연결부십자가-->


                                        <geom name = "handle_contact0" type="mesh"  mesh="handle0" class="collision" />
                                        <geom name = "handle_contact1" type="mesh"  mesh="handle1" class="collision" />
                                        <geom name = "handle_contact2" type="mesh"  mesh="handle2" class="collision"/>
                                        <geom name = "handle_contact3" type="mesh"  mesh="handle3" class="collision"/>
                                        <geom name = "handle_contact5" type="mesh"  mesh="handle5" class="collision" />
                                        <geom name = "handle_contact6" type="mesh"  mesh="handle6" class="collision"/>
                                        <geom name = "handle_contact8" type="mesh"  mesh="handle8" class="collision" /> 
                                        <geom name = "handle_contact10" type="mesh" mesh="handle10" class="collision"/>
                                        <geom name = "handle_contact11" type="mesh" mesh="handle11" class="collision"/>
                                        <geom name = "handle_contact12" type="mesh" mesh="handle12" class="collision" /> 
                                        <geom name = "handle_contact15" type="mesh" mesh="handle15" class="collision"/>
                                        <geom name = "handle_contact16" type="mesh" mesh="handle16" class="collision" /> 
                                        <geom name = "handle_contact21" type="mesh" mesh="handle21" class="collision"/>
                                        <geom name = "handle_contact22" type="mesh" mesh="handle22" class="collision"/>
                                        <geom name = "handle_contact23" type="mesh" mesh="handle23" class="collision"/>

                                    </body>
                                </body>
                            </body>
                        </mujocoinclude>
                    '''

            # Now you can write the XML content to a file
            with open(
                    '../model/mjinclude_handle.xml',
                    'w') as file:
                file.write(handle_limit_xml)
        elif obj == "valve":
            handle_xml = f'''
                     <mujocoinclude>
                         <compiler angle="radian" meshdir="meshes/" autolimits="true"/>
                         <!-- <compiler angle="radian" meshdir="meshes/"> -->
                        <size njmax="500" nconmax="100" />
                         <visual>
                            <global offwidth="3024" offheight="1680" />
                            <quality shadowsize="4096" offsamples="8" />
                            <map force="0.1" fogend="5" />
                        </visual>


                      <asset>



                        <mesh name="valve_base" file="objects/valve_base.STL" scale="{s} {s} {s}"/>
                        <mesh name="valve_base0" file="objects/valve_base/valve_base000.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve_base1" file="objects/valve_base/valve_base001.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve_base2" file="objects/valve_base/valve_base002.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve_base3" file="objects/valve_base/valve_base003.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve_base4" file="objects/valve_base/valve_base004.obj" scale="{s} {s} {s}"/>

                        <mesh name="valve" file="objects/valve.STL" scale="{s} {s} {s}"/>
                        <mesh name="valve0" file="objects/valve/valve000.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve1" file="objects/valve/valve001.obj" scale="{s} {s} {s}"/>
                        <mesh name="valve2" file="objects/valve/valve002.obj" scale="{s} {s} {s}"/>


                      </asset>

                      <contact>
                          <exclude name="valve_contact" body1="valve_base" body2="valve_handle"/>
                      </contact>


                     </mujocoinclude>
                    '''

            # Now you can write the XML content to a file
            with open('../model/assets_valve.xml',
                      'w') as file:
                file.write(handle_xml)

            handle_limit_xml = f'''
                                   <mujocoincldue>
                                        <body name="base_v" pos="0 0 0">
                                        <inertial diaginertia="0 0 0" mass="0" pos="0 0 0"/>
                                        <body name="valve_base" pos="0 0 0">
                                        <inertial pos="0 0 0" mass="2.7" diaginertia="0.1 0.1 0.1"/>
                                        <geom name="valve_base" type="mesh" rgba="1 1 1 1" mesh="valve_base" class="visual"/>
                                        <geom name="obj_contact7" type="mesh" mesh="valve_base0" class="collision"/>
                                        <geom name="obj_contact8" type="mesh" mesh="valve_base1" class="collision"/>
                                        <geom name="obj_contact9" type="mesh" mesh="valve_base2" class="collision"/>
                                        <geom name="obj_contact10" type="mesh" mesh="valve_base3" class="collision"/>
                                        <geom name="obj_contact11" type="mesh" mesh="valve_base4" class="collision"/>
                                        <body name="valve_handle" pos="0 0 0">
                                        <inertial pos="0 0 0" quat="0.365653 0.605347 -0.36522 0.605365" mass="0.1" diaginertia="0.0483771 0.0410001 0.0111013"/>
                                        <joint name="valve_joint" pos="0 0 0" axis="0 0 1" range="-{m} {m}" frictionloss="{f}" damping="0" limited="true" springref="0" stiffness="0"/>
                                        <geom name="valve" type="mesh" rgba="1 1 0 1" mesh="valve" class="visual"/>
                                        <geom name="valve_contact1" type="mesh" mesh="valve0" class="collision"/>
                                        <geom name="obj_contact13" type="mesh" mesh="valve1" class="collision"/>
                                        <geom name="valve_contact0" type="mesh" mesh="valve2" class="collision"/>
                                        </body>
                                        </body>
                                        </body>
                                    </mujocoincldue> 
                                '''

            # Now you can write the XML content to a file
            with open(
                    '../model/mjinclude_valve.xml',
                    'w') as file:
                file.write(handle_limit_xml)

        return s, m

class valve_env1(valve_template):
    def _reward(self, action_rotation, action_force):
        reward_force = 0
        reward_rotation = 1
        q_max = max(abs(self.obs_q))
        if q_max > 0.9:
            if action_force == -1:
                reward_force += 1

        if 0.7 <= abs(self.obs_omega) <= 0.8:
            reward_force += 1

        if self.contact_done:
            idx = np.where(np.array(self.contact_list) == -1)
            contact_force = 0.0
            for i in idx[0]:
                contact_force += self.data.contact[i].dist
            reward_rotation += np.log(-contact_force) * 0.01
        elif self.bound_done:
            reward_rotation -= 100
            reward_force -= 100
        elif self.time_done:
            reward_rotation += 10
            reward_force += 10

        reward_acc = -sum(abs(action_rotation - self.action_rotation_pre))

        return reward_rotation + reward_acc, reward_force

    def _done(self, end_effector):

        self.contact_list = tools.detect_contact(self.data.contact, self.desired_contact_bid)
        self.grasp_list = tools.detect_grasp(self.data.contact, self.obj, self.desired_contact_finger_bid,
                                             self.desired_contact_obj_bid)
        self.q_operation_list = tools.detect_q_operation(self.data.qpos, self.q_range)

        self.contact_done = -1 in self.contact_list
        self.bound_done = -1 in self.q_operation_list
        normalized_q = self.obs_q
        if max(abs(normalized_q)) > 0.98:
            self.bound_done = 1
        else:
            self.bound_done = 0

        if self.control_mode != RL_CIRCULAR_CONTROL:
            self.deviation_done = False
        else:
            if len(self.grasp_list) <= 2:
                self.deviation_done = True
            distance = abs(np.linalg.norm(end_effector[1][:3] - self.valve_position) - self.radius)
            if distance >= 0.005:  # 0.005
                self.deviation_done = True

        self.time_done = self.data.time - self.start_time >= self.episode_time

        if self.train:
            if self.time_done or self.bound_done or self.deviation_done or self.contact_done:
                # print(self.control_mode)
                # if self.contact_done :
                #     print("contact detected")
                # print("contact :", self.contact_done, "  //joint :", self.bound_done, "  //time :", self.time_done)
                # print("epispde time : ",self.episode_time, "time:",self.data.time-self.start_time)
                # np.save("/home/kist-robot2/catkin_ws/src/franka_overall/py_src/m0.npy", self.manipulability)
                # np.save("action.npy", self.action_data)
                return True
            else:
                return False
        else:
            if self.time_done or self.bound_done or self.deviation_done :
                return True
            else:
                return False

    def _rescale_action(self, action_force):
        if action_force < -0.666:
            action_force = -10
        elif action_force < -0.333:
            action_force = -1
        elif action_force <= 0.333:
            action_force = 0
        else:
            action_force = 1
        return action_force
