import matplotlib.pyplot as plt
import numpy as np

###########################################################
### Function to save visualization of Franka trajectory ###
###########################################################
def visualize_franka_traj(qq, save_path="franka_traj.gif", fps=20):
    import pybullet as p
    import pybullet_data
    import numpy as np
    import imageio

    # 1. Headless mode
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    p.loadURDF("plane.urdf")
    frankaId = p.loadURDF("franka_panda/panda.urdf", [0,0,0], p.getQuaternionFromEuler([0,0,0]), useFixedBase=True)

    frames = []

    # Camera setup
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.5, 0, 0.5],
        distance=1.5,
        yaw=45,
        pitch=-35,
        roll=0,
        upAxisIndex=2
    )
    proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=10.0)

    # 2. Simulate and capture frames
    for i, qi in enumerate(qq):
        print(f"rendering progress: {i+1}/{len(qq)}\t\t", end='\r')
        p.setJointMotorControlArray(frankaId, range(7), p.POSITION_CONTROL, targetPositions=qi)
        p.stepSimulation()

        width, height, rgba, _, _ = p.getCameraImage(
            width=2*640, height=2*480,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix
        )

        frame = np.reshape(rgba, (height, width, 4))[:, :, :3]  # keep RGB only
        frame = frame.astype(np.uint8)  # convert from int64 to uint8
        frames.append(frame)

    p.disconnect()

    # 3. Save to GIF or MP4
    imageio.mimsave(save_path, frames, fps=fps)
    print(f"Saved trajectory to {save_path}")






from impact import *
import casadi as cs

## Define MPC object
T = 1.5
N = 10
mpc = MPC(T=T)

## Constants
ndof = 7
nq = 7
nqdot = 7
nqddot = 7
joint_pos_ub = np.array([ 2.8973,  1.7628,  2.8973,  0.0698,  2.8973,  3.7525,  2.8973])
joint_pos_lb = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
joint_vel_ub = np.array([2.175,    2.175,   2.175,   2.175,   2.61,    2.61,    2.61 ])
joint_vel_lb = np.array([-2.175,  -2.175,  -2.175,  -2.175,  -2.61,   -2.61,   -2.61 ])
joint_acc_ub = np.array([15.,      15.,     15.,     15.,     20.,     20.,     20.])
joint_acc_lb = np.array([-15.,    -15.,    -15.,    -15.,    -20.,    -20.,    -20.])

## States and controls
q = mpc.state('q', 7)           # Joint positions
qd = mpc.state('qd', 7)      # Joint velocities
qdd =mpc.control('qdd', 7)  # Joint accelerations

## Model
mpc.set_der(q, qd)
mpc.set_der(qd, qdd)

## Parameters
x_current = mpc.parameter('x_current', nq+nqdot)
targetConfig = mpc.parameter('targetConfig', nq+nqdot)
weights = mpc.parameter('weights', 1)

## Objective
mpc.add_objective(mpc.at_tf(cs.sumsqr(cs.vertcat(q,qd) - targetConfig)) + 
                  weights*mpc.integral(cs.sumsqr(qdd)))

## Boundary constraints                                     
mpc.subject_to(mpc.at_t0(cs.vertcat(q,qd))==x_current)

## Path constraints
mpc.subject_to(joint_pos_lb  <= (q <= joint_pos_ub), include_first=False, include_last=False)
mpc.subject_to(joint_vel_lb  <= (qd <= joint_vel_ub), include_first=False, include_last=False)
mpc.subject_to(joint_acc_lb  <= (qdd <= joint_acc_ub))

## Solver
solver_name = "fatrop"
options = {solver_name: {"tol":1e-4, "max_iter":100, "print_level":0}}
options["structure_detection"] = "auto"
options["debug"] = True
options["expand"] = True
options["print_time"] = True
mpc.solver(solver_name, options)

mpc.set_value(x_current, [0,0,0,0,0,0,0,0,0,0,0,0,0,0])
mpc.set_value(targetConfig, [1.4,1.0,1.0,1.0,1.0,1.0,1.0,0,0,0,0,0,0,0])
mpc.set_value(weights, 1e-2)

## Make it concrete for this ocp
mpc.method(MultipleShooting(N=N,M=1,intg='rk'))




## simulate
dt = T/N
qq_simulated = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
t_comps = []
T_sim = 14
switched = False
for i in range(int(T_sim/dt)):
    print(f"simulation progress: {i+1}/{int(T_sim/dt)}\t\t", end='\r')

    # solve MPC problem
    sol = mpc.solve()

    # register computation time
    t_comps.append(sol.stats['t_wall_total'])

    # update the current state parameter and store travelled trajectory
    q0, qnext = sol.sample(cs.vertcat(q,qd), grid='integrator')[1][0:2]
    mpc.set_value(x_current, qnext)
    qq_simulated.append(q0.tolist())

    # switch target configuration halfway through the simulation
    if (not switched) and (i*dt > T_sim/2):
        mpc.set_value(targetConfig, [0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        switched = True

print(f"Average solve time: {np.mean(t_comps)} s")
print(f"Max solve time: {np.max(t_comps)} s")

# extract and visualize trajectory
qq = [np.array(qi).flatten() for qi in qq_simulated]
qq = [np.array(qi[0:7]) for qi in qq]
visualize_franka_traj(qq)
