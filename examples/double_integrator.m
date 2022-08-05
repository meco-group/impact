addpath(char(py.impact.matlab_path))

import impact.*

mpc = MPC('T',5.25);

% Define two scalar states (vectors and matrices also supported)
x1 = mpc.state();
x2 = mpc.state();

x = [x1;x2];
u = mpc.control();

mpc.set_der(x1, x2);
mpc.set_der(x2, u);

mpc.add_objective(mpc.integral(0.1*u^2));
mpc.add_objective(mpc.at_tf(mpc.T));

x2b = mpc.parameter('x2b');
mpc.set_value(x2b, 0.25);

mpc.subject_to(x2 <= x2b);
mpc.subject_to(-1 <= u <= 1 );

x_current = mpc.parameter('x_current',2);
mpc.set_value(x_current, [-1;-1]);

% Boundary constraints
mpc.subject_to(mpc.at_t0(x) == x_current);

mpc.subject_to(mpc.at_tf(x1) == 0);
mpc.subject_to(mpc.at_tf(x2) == 0);

% Pick an NLP solver backend
%  (CasADi `nlpsol` plugin):
mpc.solver('ipopt');

grampc_options = struct;
grampc_options.MaxGradIter = 20;
grampc_options.MaxMultIter = 30;
grampc_options.ShiftControl = 'off';
grampc_options.Integrator = 'euler';
grampc_options.LineSearchMax = 1e2;

grampc_options.PenaltyMin = 1e1;
grampc_options.PenaltyIncreaseFactor = 1.25;
grampc_options.PenaltyDecreaseFactor = 1.0;

grampc_options.ConstraintsAbsTol = 1e-3;
grampc_options.ConvergenceCheck = 'on';
grampc_options.ConvergenceGradientRelTol = 1e-3;

% Pick a solution method
method = external_method('grampc','N',40,'grampc_options',grampc_options);
%method = MultipleShooting('N',40)
mpc.method(method);

mpc.solve_limited();

mpc.export('double_integrator');
