

from impact import *
from numpy import *
import casadi as cs


# -------------------------------------------------
# Instantiating MPC object
# -------------------------------------------------
mpc = MPC(T=2.0)

# Model parameters
M, m, L, g = 0.5, 1.0, 2.0, 9.81

# -------------------------------------------------
# States and control
# -------------------------------------------------
pos    = mpc.state()
theta  = mpc.state()
dpos   = mpc.state()
dtheta = mpc.state()

x = cs.vertcat(pos, theta, dpos, dtheta)
F = mpc.control()

# -------------------------------------------------
# Dynamics
# -------------------------------------------------
mpc.set_der(pos, dpos)
mpc.set_der(theta, dtheta)
mpc.set_der(dpos,
    (-m*L*sin(theta)*dtheta**2 + m*g*cos(theta)*sin(theta)+F)
    /(M + m - m*cos(theta)**2) 
    )
# mpc.set_der(dtheta, (-m*L*cos(theta)*sin(theta)*dtheta**2 + F*cos(theta)+(M+m)*g*sin(theta))/(L*(M + m - m*cos(theta)**2)))
mpc.set_der(dtheta,
    (-m*L*cos(theta)*sin(theta)*dtheta**2 + F*cos(theta)+(M+m)*g*sin(theta))
    / (L*(M+m-m * cos(theta) ** 2))
    )

# -------------------------------------------------
# Parameters
# -------------------------------------------------
x_current = mpc.parameter("x_current", 4)
x_end     = mpc.parameter("x_end", 4)
weights   = mpc.parameter("weights", 2)

mpc.set_value(x_current, [0.5, 0, 0, 0])
mpc.set_value(x_end, [0, 0, 0, 0])
mpc.set_value(weights, [100, 1])

# -------------------------------------------------
# Objective
# -------------------------------------------------
mpc.add_objective(
    mpc.integral(weights[0] * pos**2 + weights[1] * F**2)
)

# equivalent to the above
# mpc.add_objective(mpc.integral(weights[0]*F**2 ))
# mpc.add_objective(mpc.integral(weights[1]*100*pos**2 ))

# -------------------------------------------------
# Constraints
# -------------------------------------------------
mpc.subject_to(mpc.at_t0(x) == x_current)
mpc.subject_to(mpc.at_tf(x) == x_end)

mpc.subject_to(-2 <= (F <= 2))
mpc.subject_to(-2 <= (pos <= 2), include_first=False)

# -------------------------------------------------
# Transcription
# -------------------------------------------------
method = MultipleShooting(N=40,M=2)
# method = SingleShooting(N=40, M=2
# method = DirectCollocation(N=40,M=2)
mpc.method(method)

# -------------------------------------------------
# Solver
# -------------------------------------------------

# options = {"ipopt": {"print_level": 0}}
# options["expand"] = True
# options["print_time"] = False
# mpc.solver('ipopt',options)

options = {
    "expand": True,
    "structure_detection": "auto",
    "print_time": False,
    "fatrop.print_level": 0,
}
mpc.solver("fatrop", options)


# -------------------------------------------------
# Solve
# -------------------------------------------------
sol = mpc.solve()

# -------------------------------------------------
# Sample solution
# -------------------------------------------------
tsol, posSol   = sol.sample(pos, grid="control")
_,    thetaSol = sol.sample(theta, grid="control")
tsolu, Fsol    = sol.sample(F, grid="control")

# -------------------------------------------------
# Plot results
# -------------------------------------------------

from pylab import *
import matplotlib as mpl
mpl.rcParams["text.usetex"] = False
mpl.rcParams["font.family"] = "serif"
mpl.rcParams["mathtext.fontset"] = "cm"

fig = plt.figure(figsize=(8, 6))
ax1 = plt.subplot(2, 1, 1)
ax2 = plt.subplot(2, 1, 2, sharex=ax1)

ax1.plot(tsol, posSol,   marker="o", linewidth=2, label="cart position [m]")
ax1.plot(tsol, thetaSol, marker="o", linewidth=2, label="pendulum angle [rad]")
ax1.set_ylabel("States")
ax1.grid(True)
ax1.legend()

for label in ax1.get_xticklabels():
    label.set_visible(False)

ax2.plot(tsolu,  2 * ones(Fsol.shape[0]), "--", color="gray", linewidth=2)
ax2.plot(tsolu, -2 * ones(Fsol.shape[0]), "--", color="gray", linewidth=2, label="saturation")
ax2.step(tsolu, Fsol, color="brown", linewidth=2, label="control signal [N]")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Input")
ax2.grid(True)
ax2.legend()
ax2.legend(loc='upper right', fontsize=12, bbox_to_anchor=(0.82, 0.8), borderaxespad=0)

plt.tight_layout()

show(block=True)


#######################################################################
# Simulating the MPC controller
########################################################################
Ts = 2/40          # sampling time (horizon/# samples)
Tsim = 3.5         # simulation time
samples = int(np.floor(Tsim / Ts))  # number of simulation steps

# NOTE: need samples+1 because we store x(k) ... x(samples)
xlog = np.zeros((4, samples + 1))
ulog = np.zeros((1, samples))
t = np.zeros(samples + 1)

xlog[:, 0] = [0.5, 0, 0, 0]

# Get the discrete model as CasADi function
Sim_pendulum_dyn = mpc._method.discrete_system(mpc)

for i in range(samples):
    # Set current state
    mpc.set_value(x_current, xlog[:, i])

    # Solve OCP
    sol = mpc.solve()
    _, usol = sol.sample(F, grid="control")

    # Apply first input and simulate one step
    current_X = Sim_pendulum_dyn(x0=xlog[:, i], u=usol[0], T=Ts)["xf"]

    # Log next state and input
    xlog[:, i + 1] = current_X.full().squeeze()
    ulog[:, i] = usol[0]
    t[i + 1] = (i + 1) * Ts



# --- Plot simulated states ---
plt.figure()
plt.plot(t, xlog[0, :], marker="*", markersize=5, label="pos")
plt.plot(t, xlog[1, :], marker="*", markersize=5, label="theta")
plt.plot(t, xlog[2, :], marker="*", markersize=5, label="dpos")
plt.plot(t, xlog[3, :], marker="*", markersize=5, label="dtheta")
plt.xlabel("Time [s]")
plt.ylabel("States")
plt.title("Simulated closed-loop states")
plt.grid(True)
plt.legend()


# --- Plot simulated control ---
plt.figure()
plt.step(t[:-1], ulog[0, :])
plt.title("Control signal")
plt.ylabel("Force [N]")
plt.xlabel("Time [s]")
plt.grid(True)

show(block=True)


######################################################################
# Exportation (create the Impact artifacts)
########################################################################

# mpc.export('cart_pend', short_output=True)
mpc.export('cart_pendros', short_output=True, ros2=True)




