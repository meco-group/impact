import dirac_mpc.*
import casadi.*

mpc = MPC('T',2.0);

cart_pendulum = mpc.add_model('cart_pendulum','cart_pendulum.yaml');

% Parameters
final_X = mpc.parameter('final_X',cart_pendulum.nx)
x_current = mpc.parameter('x_current',cart_pendulum.nx)

% Objectives
mpc.add_objective(mpc.integral(cart_pendulum.F^2 + 100*cart_pendulum.pos^2))

% Path constraints
mpc.subject_to(-2 <= (cart_pendulum.F <= 2 ))
mpc.subject_to(-2 <= (cart_pendulum.pos <= 2))

% Initial constraints
mpc.subject_to(mpc.at_t0(cart_pendulum.x)==x_current)
mpc.subject_to(mpc.at_tf(cart_pendulum.x)==final_X)

% Solver
options = struct;
options.ipopt.print_level = 5;
options.expand = true;
options.print_time = false;

mpc.solver('ipopt',options);

mpc.set_value(x_current, [0.5,0,0,0])
mpc.set_value(final_X, [0,0,0,0])

% Make it concrete for this ocp
mpc.method(MultipleShooting('N',50,'M',1,'intg','rk'))

mpc.export('cart_pendulum')


%impact = Impact('cart_pendulum');
%impact.print_problem();
%impact.solve();
