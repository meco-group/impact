from dirac_mpc import *

mpc = MPC(T=2.0)

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum2.yaml')

# Parameters
x_current = mpc.parameter('x_current',cart_pendulum.nx)
x_final = mpc.parameter('x_final',cart_pendulum.nx)
weights = mpc.parameter('weights',2)

# Objectives
mpc.add_objective(mpc.integral(weights[0]*cart_pendulum.F**2 + weights[1]*100*cart_pendulum.pos**2))

# Path constraints
mpc.subject_to(-2 <= (cart_pendulum.F <= 2 ))
mpc.subject_to(-2 <= (cart_pendulum.pos <= 2))

# Boundary constraints
mpc.subject_to(mpc.at_t0(cart_pendulum.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pendulum.x)==x_final)

# Solver
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
mpc.solver('ipopt',options)

mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(x_final, [0,0,0,0])
mpc.set_value(weights, [1,1])

# Make it concrete for this ocp
mpc.method(MultipleShooting(N=50,M=1,intg='rk'))

mpc.export("cart_pendulum")


