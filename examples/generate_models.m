import casadi.*

pos = MX.sym('pos');
theta = MX.sym('theta');
dpos = MX.sym('dpos');
dtheta = MX.sym('dtheta');

F = MX.sym('F');

mcart = 0.5;   % cart mass      [kg]
m = 1;         % pendulum mass  [kg]
L = 2;         % pendulum length [m]
g = 9.81;      % gravitation [m/s^2]

pos_rhs = dpos;
theta_rhs = dtheta;
dpos_rhs = (-m*L*sin(theta)*dtheta*dtheta + m*g*cos(theta)*sin(theta)+F)/(mcart + m - m*cos(theta)*cos(theta));
dtheta_rhs = (-m*L*cos(theta)*sin(theta)*dtheta*dtheta + F*cos(theta)+(mcart+m)*g*sin(theta))/(L*(mcart + m - m*cos(theta)*cos(theta)));

dae = struct;
dae.x = [pos;theta;dpos;dtheta];
dae.u = F;
dae.ode = [pos_rhs;theta_rhs;dpos_rhs;dtheta_rhs];

model = Function('model',dae,{'x','u'},{'ode','dae','quad'});
model.save('cart_pendulum_equations.casadi')
