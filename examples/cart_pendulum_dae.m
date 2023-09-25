addpath(char(py.impact.matlab_path))

import impact.*

mpc = MPC();

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum_dae.yaml');

% Parameters
x_current = mpc.parameter('x_current',cart_pendulum.nx);
x_final = mpc.parameter('x_final',cart_pendulum.nx);
weights = mpc.parameter('weights',2);

% Make the time horizon a parameter
T = mpc.parameter('T');
mpc.set_value(T,2.0);
mpc.set_T(T);

% Objectives
mpc.add_objective(mpc.integral(weights(1)*cart_pendulum.U^2 + weights(2)*100*cart_pendulum.X^2))

% Initial constraints
mpc.subject_to(mpc.at_t0(cart_pendulum.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pendulum.x)==x_final)

% Path constraints
mpc.subject_to(-2 <= cart_pendulum.U <= 2 )
% In MPC, you typically do not want to enforce state constraints at the initial time
mpc.subject_to(-2 <= cart_pendulum.X <= 2, 'include_first', false)

% Solver
%options = struct;
%options.ipopt.print_level = 0;
%options.expand = true;
%options.print_time = false;
%mpc.solver('ipopt',options);

options = struct;
options.qpsol = 'qrqp';
mpc.solver('sqpmethod',options)


mpc.set_value(x_current, [0.5,cart_pendulum.L,0,0,0,0])
mpc.set_value(x_final, [0,cart_pendulum.L,0,0,0,0])
mpc.set_value(weights, [1,1])

mpc.set_initial(cart_pendulum.xa, 1.11)
mpc.set_initial(cart_pendulum.x, 1.13)

% Make it concrete for this ocp
mpc.method(DirectCollocation('N',50))

% Demonstrate how to export helper functions

x = casadi.MX.sym('x');
y = casadi.MX.sym('y',3);
z = casadi.MX.sym('z',3,3);
foo = casadi.Function('foo',{x,y,z},{2*x,x.*y,z*y+x},{'x','y','z'},{'foo','bar','baz'});
mpc.add_function(foo) % Becomes available in library*.slx


mpc.export('cart_pendulum')
