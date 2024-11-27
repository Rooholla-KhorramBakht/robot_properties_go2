"""Go2 wrapper

Go2 pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import pybullet
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_go2.config import Go2Config

dt = 1e-3


class Go2Robot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=False,
        init_sliders_pose=4
        * [
            0,
        ],
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.3]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(Go2Config.meshes_path)
        self.urdf_path = Go2Config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        pybullet.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        self.pin_robot = Go2Config.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []
        eff_names = ["FR", "FL", "RR", "RL"]

        # TODO: This parts need to be redone
        for ee in eff_names:
            controlled_joints += [ee + "_hip_joint", ee + "_thigh_joint", ee + "_calf_joint"]
            self.end_effector_names.append(ee + "_foot_joint")

        print(controlled_joints)
        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        self.lh_index = self.pin_robot.model.getFrameId("RL_foot")
        self.rh_index = self.pin_robot.model.getFrameId("RR_foot")
        self.lf_index = self.pin_robot.model.getFrameId("FL_foot")
        self.rf_index = self.pin_robot.model.getFrameId("FR_foot")
        # Creates the wrapper by calling the super.__init__.
        super(Go2Robot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            self.end_effector_names,
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def get_slider_position(self, letter):
        if letter == "a":
            return pybullet.readUserDebugParameter(self.slider_a)
        if letter == "b":
            return pybullet.readUserDebugParameter(self.slider_b)
        if letter == "c":
            return pybullet.readUserDebugParameter(self.slider_c)
        if letter == "d":
            return pybullet.readUserDebugParameter(self.slider_d)
