addpath(char(py.impact.matlab_path))

import impact.*

mpc = MPC('T',2.0);

cart_pend = mpc.add_model('cart_pendulum','cart_pendulum.yaml');

% Parameters
x_current = mpc.parameter('x_current',cart_pend.nx);
x_final = mpc.parameter('x_final',cart_pend.nx);
weights = mpc.parameter('weights',2);

% Objectives
mpc.add_objective(mpc.integral(weights(1)*cart_pend.F^2 + weights(2)*100*cart_pend.pos^2))

% Initial constraints
mpc.subject_to(mpc.at_t0(cart_pend.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pend.x)==x_final)

% Path constraints
mpc.subject_to(-2 <= cart_pend.F <= 2 )
% In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= cart_pend.pos <= 2, 'include_first', false)


% Solver
options = struct;
options.ipopt.print_level = 0;
options.expand = true;
options.print_time = false;

mpc.solver('ipopt',options);

mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(x_final, [0,0,0,0])
mpc.set_value(weights, [1,1])

% Make it concrete for this ocp
mpc.method(MultipleShooting('N',50,'M',1,'intg','rk'))

mpc.export('cart_pend')


mpc.save('cart_pendulum.impact');

mpc2 = MPC.load('cart_pendulum.impact');
mpc2.export('cart_pend2')
