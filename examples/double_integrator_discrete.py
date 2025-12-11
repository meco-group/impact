from impact import *

mpc = MPC(T=2.0)

import numpy as np
import scipy.signal as signal

####################
# Model
####################
# double integrator
# Define the continuous-time system matrices
Ac = np.array([[0, 1], [0, 0]])
Bc = np.array([[0], [1]])
Cc = np.array([[1, 0], [0, 1], [0, 0]])
Dc = np.array([[0], [0], [1]])
Ts = 0.1

# Create the continuous-time state-space system
sysc = signal.StateSpace(Ac, Bc, Cc, Dc)

# Discretize the continuous-time system using zero-order hold
sysd = sysc.to_discrete(Ts, method='zoh')

# Extract the discrete-time system matrices
Ad = sysd.A
Bd = sysd.B
Cd = sysd.C
Dd = sysd.D

# Print the discrete-time system matrices
print("Ad:", Ad)
print("Bd:", Bd)
print("Cd:", Cd)
print("Dd:", Dd)


xT = mpc.state(2, 1) # x = [s, \phi, \beta, \omega]^T
dfT = mpc.control(1, 1); # steering

# Vehicle linear dynamics, space state representation
rhs = Ad@xT + Bd@dfT

mpc.set_next(xT,rhs)


# Parameters
x_current = mpc.parameter('x_current',2)
x_final = mpc.parameter('x_final',2)
weights = mpc.parameter('weights',2)

# Objectives
mpc.add_objective(mpc.sum(weights[0]*dfT**2 + weights[1]*100*xT[0]**2))

# Initial constraints
mpc.subject_to(mpc.at_t0(xT) == x_current)
mpc.subject_to(mpc.at_tf(xT) == x_final)

# Path constraints
mpc.subject_to(-2 <= (dfT <= 2))
# In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= (xT[1] <= 2), include_first= False)

# Solver
options = {"ipopt": {"print_level": 3}}
options["expand"] = True
options["print_time"] = False
options["error_on_fail"] = True
mpc.solver('ipopt',options)


mpc.set_value(x_current, [0.5,0])
mpc.set_value(x_final, [0,0])
mpc.set_value(weights, [1,1])

# Make it concrete for this ocp
mpc.method(MultipleShooting(N=50,M=1,intg='rk'))

# ## Export artifacts from the MPC object
mpc_name = "my_mpc"
mpc.export(mpc_name)


# mpc.export(mpc_name, ros2=True)

#  mpc.solve()
