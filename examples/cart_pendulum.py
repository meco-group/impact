"""
Cart-pendulum
=============

Basic cart-pendulum example using YAML model
"""
from impact import *

mpc = MPC(T=2.0)

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum2.yaml')

## Parameters
x_current = mpc.parameter('x_current',cart_pendulum.nx)
x_final = mpc.parameter('x_final',cart_pendulum.nx)
weights = mpc.parameter('weights',2)

## Objectives
mpc.add_objective(mpc.sum(weights[0]*cart_pendulum.F**2 + weights[1]*100*cart_pendulum.pos**2))

## Boundary constraints
mpc.subject_to(mpc.at_t0(cart_pendulum.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pendulum.x)==x_final)

## Path constraints
mpc.subject_to(-2 <= (cart_pendulum.F <= 2 ))
## In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= (cart_pendulum.pos <= 2), include_first=False)

## Solver
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
mpc.solver('ipopt',options)

mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(x_final, [0,0,0,0])
mpc.set_value(weights, [1,1])

## Make it concrete for this ocp
mpc.method(MultipleShooting(N=50,M=1,intg='rk'))

#mpc.method(external_method('fatrop',N=50,mode='interface',fatrop_options={"tol":1e-5}))

## Save the MPC object
# mpc.save("cart_pendulum.impact")
## Export artifacts from the MPC object
# mpc.export("cart_pendulum", qp_error_on_fail=False)


