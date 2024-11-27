import numpy as np
import pinocchio as pin
import pybullet as p
from bullet_utils.env import BulletEnvWithGround

from robot_properties_go2.go2wrapper import Go2Robot
from robot_properties_go2.config import Go2Config


def main():
    """Simulates a standing controller"""

    # Create a Pybullet simulation environment before any robots
    env = BulletEnvWithGround()

    # Remove debug sliders
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create a robot instance. This adds the robot to the simulator as well.
    robot = Go2Robot(useFixedBase=False)

    # Add the robot to the env to update the internal structure of the robot
    # at every simulation steps.
    env.add_robot(robot)

    # Reset the robot to some initial state.
    q0 = np.matrix(Go2Config.initial_configuration).T
    dq0 = np.matrix(Go2Config.initial_velocity).T
    robot.reset_state(q0, dq0)

    leg_gc_force = -np.array([[0.0], [0.0], [Go2Config.mass * 9.81 / 4]])

    # print("mass: {}".format(go2Config.mass))

    # Run the simulator for 1e4 steps
    for _ in range(10000):
        q, dq = robot.get_state_update_pinocchio()
        gravity = robot.pin_robot.gravity(q)

        tau = (
            gravity[6:]
            + 200.0 * (np.asarray(q0)[7:, 0] - q[7:])
            + 20.0 * (np.asarray(dq0)[6:, 0] - dq[6:])
        )

        # Check contact status
        contact_status, _ = robot.end_effector_forces()

        # # Once there is contact the robot exerts force to support the body
        if np.sum(contact_status) > 1e-3:
            for idx in [robot.lf_index, robot.rf_index, robot.lh_index, robot.rh_index]:
                _J = robot.pin_robot.getFrameJacobian(
                    idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )[:3, 6:]
                _tau_gc = _J.T @ leg_gc_force
                tau = tau + _tau_gc[:, 0]

        # τ = G(q) + Kp(q_des - q) + Kd(q̇_des - q̇) + JᵀF
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(sleep=True)


if __name__ == "__main__":
    main()
