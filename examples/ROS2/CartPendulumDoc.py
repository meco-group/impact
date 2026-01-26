

from impact import *
from numpy import *
import casadi as cs




#Instantiating MPC object
mpc = MPC(T = 2.0)


# model parameters
# M = 0.5   # cart mass [kg]
# m =     1     # pendulum mass [kg]
# L =     2     # pendulum length [m]
# g =     9.81  # gravitation [m/s^2]

M, m, L, g = 0.5, 1, 2, 9.81  # cart mass [kg], pendulum mass [kg], pendulum length [m], gravity [m/s^2]


## Defining states and control input
pos = mpc.state()
theta = mpc.state()
dpos = mpc.state()
dtheta = mpc.state()

x = cs.vertcat(pos,theta,dpos,dtheta)
F = mpc.control()

# Defining the dynamics of the system
mpc.set_der(pos, dpos)
mpc.set_der(theta, dtheta)
# mpc.set_der(dpos,(-m*L*sin(theta)*dtheta**2 + m*g*cos(theta)*sin(theta)+F)/(M + m - m*cos(theta)**2) )
mpc.set_der(dpos,
    (-m*L*sin(theta)*dtheta**2 + m*g*cos(theta)*sin(theta)+F)
    /(M + m - m*cos(theta)**2) 
    )
# mpc.set_der(dtheta, (-m*L*cos(theta)*sin(theta)*dtheta**2 + F*cos(theta)+(M+m)*g*sin(theta))/(L*(M + m - m*cos(theta)**2)))
mpc.set_der(dtheta,
    (-m*L*cos(theta)*sin(theta)*dtheta**2 + F*cos(theta)+(M+m)*g*sin(theta))
    / (L*(M+m-m * cos(theta) ** 2))
    )

# Defining parameters for the MPC
x_current = mpc.parameter('x_current',4)
x_end = mpc.parameter('x_end',4)
weights = mpc.parameter('weights',2)

# Setting initial values for the parameters (for solving here in code, after the exportation should be defined in the new environment)
mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(x_end, [0,0,0,0])
mpc.set_value(weights, [100,1])

# Adding objectives to the OCP (Cost function)
mpc.add_objective(mpc.integral( weights[0]*pos**2 + weights[1]*F**2 ))

# equivalent to the above
# mpc.add_objective(mpc.integral(weights[0]*F**2 ))
# mpc.add_objective(mpc.integral(weights[1]*100*pos**2 ))

# initial and final constraints
mpc.subject_to(mpc.at_t0(x)==x_current)
mpc.subject_to(mpc.at_tf(x)==x_end)

# Actuator saturation
mpc.subject_to(-2 <= (F <= 2 ))
# In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= (pos <= 2), include_first=False)

# Defining the transcription method
method = MultipleShooting(N=40,M=2)
mpc.method(method)

# selection of the solver
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


# options = {
#     "expand": True,
#     "structure_detection": "auto",
#     "fatrop.tol": 1e-4,
#     "print_time": False,
#     "fatrop.print_level": 0,
#     # "fatrop.max_iter" : 0,
#     "debug": True,
#     "common_options":{"final_options":{"cse":True}},
#     }
# mpc.solver("fatrop", options)


# solve the OCP (Optimal control problem)
sol=mpc.solve()



# Sample a state/control solution or expression on a grid
tsol, posSol = sol.sample(pos, grid='control')
tsol, thetaSol = sol.sample(theta, grid='control')
tsolu, Fsol = sol.sample(F, grid='control')

# Plotting the results
# from pylab import *
# figure(figsize=(8, 4))
# plot(tsa, x1a, marker="o",markersize=5)
# xlabel('Times [s]')
# ylabel('Position [m]')
# legend(['Control grid'])
# title('Position x(1)')
# grid(True)

# figure(figsize=(8, 4))
# plot(tsol,2*ones(Fsol.shape[0]), linestyle='--', color='gray')  # Add a dashed line at y=2 for reference
# plot(tsol,-2*ones(Fsol.shape[0]), linestyle='--', color='gray')  # Add a dashed line at y=-2 for reference
# step(tsol,Fsol, color ='brown')
# title("Control signal")
# xlabel("Times [s]")
# grid(True)

from pylab import *
import matplotlib as mpl
mpl.rcParams['text.usetex']      = False
mpl.rcParams['font.family']      = 'serif'
mpl.rcParams['mathtext.fontset'] = 'cm'


# Create one figure with two rows, one column, and share the x‐axis
figure(figsize=(8, 6))           # adjust height so both plots fit nicely
ax1 = subplot(2, 1, 1)           # top subplot
ax2 = subplot(2, 1, 2, sharex=ax1)  # bottom subplot, shares x‐axis with ax1

# --- Top plot (Position x(1)) ---
ax1.plot(tsol, posSol, marker="o", markersize=5, linewidth=2, label='cart position [m]')
ax1.plot(tsol, thetaSol, marker="o", markersize=5, linewidth=2, label='pendulum angle [rad]')
ax1.set_ylabel('States', fontsize=14)
# ax1.legend(['Control grid'])
# ax1.set_title('Position x(1)')
ax1.grid(True)
ax1.legend(loc='upper right', fontsize=12)

# Hide the x‐tick labels on the top axes,
# since we only want the “Times [s]” axis shown on the bottom plot.
for label in ax1.get_xticklabels():
    label.set_visible(False)

# --- Bottom plot (Control signal) ---
ax2.plot(tsolu,  2 * ones(Fsol.shape[0]), linestyle='--', color='gray', linewidth=2, label='_nolegend_')
ax2.plot(tsolu, -2 * ones(Fsol.shape[0]), linestyle='--', color='gray', linewidth=2, label='saturation')
# Then the step plot
ax2.step(tsolu, Fsol, color='brown', linewidth=2, label='control signal [N]')
ax2.set_xlabel('Times [s]', fontsize=14)
ax2.set_ylabel('Input', fontsize=14)
# ax2.set_title('Control signal')
ax2.grid(True)
# Make ax2 and ax1 tick‐labels bigger:
ax1.tick_params(axis='both', which='major', labelsize=12)
ax2.tick_params(axis='both', which='major', labelsize=12)
# Legend font size
ax2.legend(loc='upper right', fontsize=12, bbox_to_anchor=(0.82, 0.8), borderaxespad=0)

    

tight_layout()

show(block=True)


########################################################################
# Simulating the MPC controller
########################################################################
Ts=2/40 # sampling time (horizon/# samples)
Tsim=3.5 # simulating time
samples=int(floor(Tsim/Ts))  #numbers of samples to simulate
xlog=np.zeros([4,samples])   # variable to log states
ulog=np.zeros([1,samples])   # variable to log the input
xlog[:,0]=[0.5,0,0,0]

t=np.zeros([samples])
Sim_pendulum_dyn = mpc._method.discrete_system(mpc) # get the discrete model as CasADi function

for i in range(samples):
    mpc.set_value(x_current, xlog[:,i]) # Set the currrent state values to x_current

    sol = mpc.solve() # Solve the optimization problem
    tsol, xsol = sol.sample(x, grid='control')
    tsol, usol = sol.sample(F, grid='control')

    current_X = Sim_pendulum_dyn(x0=xlog[:,i], u=usol[0], T=Ts)["xf"] # Simulate dynamics (applying the first control input) and update the current state

    xlog[:,i+1:i+2] = current_X[:,0].full()  # Log data
    ulog[:,i]   = usol[0]  # Log data
    t[i]=i*Ts  # Time vector

figure()
plot(t, xlog[0,:], marker="*",markersize=5)
plot(t, xlog[1,:], marker="*",markersize=5)
plot(t, xlog[2,:], marker="*",markersize=5)
plot(t, xlog[3,:], marker="*",markersize=5)
xlabel('Times [s]')
ylabel('States')
legend(['pos','theta','dpos','dtheta'])
title('States')
grid(True)

figure()
step(t,ulog[0,:])
title("Control signal")
ylabel("Force [N]")
xlabel("Times [s]")
grid(True)

show(block=True)


# Export the problem (create the Impact artifacts)
# mpc.export('cart_pend', short_output=True)
mpc.export('cart_pendros', short_output=True, ros2=True)




