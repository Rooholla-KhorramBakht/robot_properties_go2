"""config

Store the configuration of the A1 robot.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
from math import pi
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_go2.utils import find_paths


class Go2Abstract(object):
    """Abstract class used for all Solo robots."""

    # PID gains
    kp = 20.0
    kd = 0.6
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.0

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 0.0

    # Maximum torques.
    max_torque = 45

    # Maximum control one can send, here the control is the current.
    max_control = max_torque

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi

    base_link_name = "base_link"


    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names

class Go2Config(Go2Abstract):
    robot_family = "unitree"
    robot_name = "go2"

    paths = find_paths(robot_name)
    meshes_path = paths["package"]
    urdf_path = paths["urdf"]
    print(meshes_path)
    print('--------------------------------')
    # The inertia of a single blmc_motor.
    motor_inertia = 0.0

    # The motor gear ratio.
    motor_gear_ration = 0.0

    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, se3.JointModelFreeFlyer()
    )
    pin_robot_wrapper.model.rotorInertia[6:] = motor_inertia
    pin_robot_wrapper.model.rotorGearRatio[6:] = motor_gear_ration
    pin_robot = pin_robot_wrapper

    robot_model = pin_robot_wrapper.model
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = "base_link"

    # End effectors informations
    shoulder_ids = []
    end_eff_ids = []
    shoulder_names = []
    end_effector_names = []

    # TODO: Needs to be redone for Atlas
    for leg in ["FR", "FL", "RR", "RL"]:
        shoulder_ids.append(robot_model.getFrameId(leg + "_hip"))
        shoulder_names.append(leg + "_hip")
        end_eff_ids.append(robot_model.getFrameId(leg + "_foot"))
        end_effector_names.append(leg + "_foot")



    joint_names = [
        "FR_HAA",
        "FR_HFE",
        "FR_KFE",
        "FL_HAA",
        "FL_HFE",
        "FL_KFE",
        "RR_HAA",
        "RR_HFE",
        "RR_KFE",
        "RL_HAA",
        "RL_HFE",
        "RL_KFE",
    ]
    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0] + 4 * [0.0, 0.77832842, -1.56065452]
    initial_velocity = int((8 + 4 + 6) / 2) * [0.0, 0.0]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)
    base_p_com = [0.0, 0.0, 0.0]
