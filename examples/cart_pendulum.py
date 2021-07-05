from dirac_mpc import *
from casadi import *

mpc = MPC(T=2.0)

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum.yaml')

# Parameters
x_current = mpc.parameter('x_current',cart_pendulum.nx)
x_final = mpc.parameter('x_final',cart_pendulum.nx)
weights = mpc.parameter('weights',2)

# Objectives
mpc.add_objective(mpc.integral(weights[0]*cart_pendulum.F**2 + weights[1]*100*cart_pendulum.pos**2))

# Path constraints
mpc.subject_to(-2 <= (cart_pendulum.F <= 2 ))
mpc.subject_to(-2 <= (cart_pendulum.pos <= 2))

# Initial constraints
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

impact = Impact("cart_pendulum")

print("Solve a single OCP (default parameters)")
impact.solve()

# Get solution trajectory
x_opt = impact.get("x_opt", impact.ALL, impact.EVERYWHERE, impact.FULL)

# Plotting
import pylab as plt

_, ax = plt.subplots(2,1,sharex=True)
ax[0].plot(x_opt.T)
ax[0].set_title('Single OCP')
ax[0].set_xlabel('Sample')

print("Running MPC simulation loop")

history = []
for i in range(100):
  impact.solve()

  # Optimal input at k=0
  u = impact.get("u_opt", impact.ALL, 0, impact.FULL)

  # Simulate 1 step forward in time
  # (TODO: use simulation model other than MPC model)
  x_sim = impact.get("x_opt", impact.ALL, 1, impact.FULL)

  # Update current state
  impact.set("x_current", impact.ALL, 0, impact.FULL, x_sim)
  history.append(x_sim)

# More plotting
ax[1].plot(np.hstack(history).T)
ax[1].set_title('Simulated MPC')
ax[1].set_xlabel('Sample')
plt.show()



