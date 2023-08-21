"""
Cart-pendulum features
======================

Basic cart-pendulum example
"""
from impact import *

from rockit import *

from casadi import sumsqr

mpc = MPC(T=FreeTime(2.0))

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum2.yaml')

# Parameters
x_current = mpc.parameter('x_current',cart_pendulum.nx)
x_final = mpc.parameter('x_final',cart_pendulum.nx)
weights = mpc.parameter('weights',2)

#w = mpc.parameter('w',cart_pendulum.nx,grid='control')

#v = mpc.variable("v")

#mpc.set_initial(v0,42)

#w = mpc.parameter('w',cart_pendulum.nx,grid='control')

#mpc.set_initial(v0,42)

print(mpc.variables)

# Objectives
mpc.add_objective(mpc.integral(weights[0]*cart_pendulum.F**2 + weights[1]*100*cart_pendulum.pos**2))

# Boundary constraints
mpc.subject_to(mpc.at_t0(cart_pendulum.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pendulum.x)==x_final)

# Path constraints
mpc.subject_to(-2 <= (cart_pendulum.F <= 2 ))
# In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= (cart_pendulum.pos <= 2), include_first=False)



# Solver
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
mpc.solver('ipopt',options)

mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(x_final, [0,0,0,0])
mpc.set_value(weights, [1,1])

#mpc.set_value(w, 3)

# Make it concrete for this ocp
mpc.method(MultipleShooting(N=50,M=1,intg='rk'))

mpc.save("cart_pendulum.impact")



mpc.export("cart_pendulum",qp_error_on_fail=False)


mpc2 = MPC.load("cart_pendulum.impact")
mpc2.export("cart_pendulum2",qp_error_on_fail=False)
