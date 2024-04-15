.. **********************
.. Getting started
.. **********************

First of all, you need to import Impact

.. tab:: Python

    .. code:: python

        from impact import *
        import casadi as cs

.. tab:: MATLAB

    .. code:: matlab

        addpath(char(py.impact.matlab_path))
        import impact.*


.. :math:`y = mx + \frac{b}{1}`

:math:`\begin{equation}
\begin{aligned}
&\underset{\boldsymbol{x},\ \boldsymbol{u}}{\text{min}}&&{\int_{t_0}^{t_f} V(\boldsymbol{x}(t),\boldsymbol{u}(t), \boldsymbol{p})\ \mathrm{d}t}\\
&\text{subject to} && \dot{\boldsymbol{x}}(t) = f(\boldsymbol{x}(t),\boldsymbol{u}(t), \boldsymbol{p}),\\
&&& g(\boldsymbol{x}(t),\boldsymbol{u}(t), \boldsymbol{p}) \leq 0.
\end{aligned}
\end{equation}`


************************
Instantiating MPC object
************************

.. tab:: Python

    .. code:: python

        mpc = MPC(T = 5.0)

.. tab:: MATLAB

    .. code:: matlab

        mpc = MPC('T', 5.0);

.. todo::
    Also option for FreeTime, ...

***********************
Defining a system model
***********************

.. todo::
    How to specify a model with Impact.

Define your system model.

.. tab:: Python

    .. code:: python

        x1 = mpc.state()
        x2 = mpc.state()

        x = cs.vertcat(x1,x2)
        u = mpc.control()

        mpc.set_der(x1, x2)
        mpc.set_der(x2, u)


.. tab:: MATLAB

    .. code:: matlab

        x1 = mpc.state();
        x2 = mpc.state();

        x = [x1;x2];
        u = mpc.control();

        mpc.set_der(x1, x2);
        mpc.set_der(x2, u);



Add model based on YAML file

.. tab:: Python

    .. code:: python

        cart_pend = mpc.add_model('cart_pendulum','cart_pendulum.yaml')

.. tab:: MATLAB

    .. code:: matlab

        cart_pend = mpc.add_model('cart_pendulum','cart_pendulum.yaml');


YAML file with inline equations

.. code:: yaml

    equations:
    inline:
        ode:
        pos: dpos
        theta: dtheta
        dpos: (-m*L*sin(theta)*dtheta*dtheta + m*g*cos(theta)*sin(theta)+F)/(mcart + m - m*cos(theta)*cos(theta))
        dtheta: (-m*L*cos(theta)*sin(theta)*dtheta*dtheta + F*cos(theta)+(mcart+m)*g*sin(theta))/(L*(mcart + m - m*cos(theta)*cos(theta)))
    differential_states: 
    - name: pos
    - name: theta
    - name: dpos
    - name: dtheta
    controls: 
    - name: F
    constants:
    inline:
        mcart: 0.5   # cart mass [kg]
        m:     1     # pendulum mass [kg]
        L:     2     # pendulum length [m]
        g:     9.81  # gravitation [m/s^2]

YAML file with external ordinary differential equations

.. code:: yaml

    equations:
    external:
        type: casadi_serialized
        file_name: cart_pendulum_equations.casadi
    differential_states: 
    - name: pos
    - name: theta
    - name: dpos
    - name: dtheta
    controls: 
    - name: F




**********************
Specifying the problem
**********************

.. todo::
    How to specify a problem with Impact.


Add objective

.. tab:: Python

    .. code:: python

        mpc.add_objective(mpc.integral(0.1*u**2))
        mpc.add_objective(mpc.at_tf(1*mpc.T))


.. tab:: MATLAB

    .. code:: matlab

        mpc.add_objective(mpc.integral(0.1*u^2));
        mpc.add_objective(mpc.at_tf(mpc.T));

Define parameters

.. tab:: Python

    .. code:: python

        x2b = mpc.parameter('x2b')
        x_current = mpc.parameter('x_current',2)

        mpc.set_value(x2b, 0.25)
        mpc.set_value(x_current, cs.vertcat(-1,-1))


.. tab:: MATLAB

    .. code:: matlab

        x2b = mpc.parameter('x2b');
        x_current = mpc.parameter('x_current',2);

        mpc.set_value(x2b, 0.25);
        mpc.set_value(x_current, [-1;-1]);


Set constraints

.. tab:: Python

    .. code:: python

        mpc.subject_to(x2 <= x2b)
        mpc.subject_to(-1 <= (u <= 1 ))

        # Boundary constraints
        mpc.subject_to(mpc.at_t0(x) == x_current)

        mpc.subject_to(mpc.at_tf(x1) == 0)
        mpc.subject_to(mpc.at_tf(x2) == 0)


.. tab:: MATLAB

    .. code:: matlab

        mpc.subject_to(x2 <= x2b);
        mpc.subject_to(-1 <= u <= 1 );

        % Boundary constraints
        mpc.subject_to(mpc.at_t0(x) == x_current);

        mpc.subject_to(mpc.at_tf(x1) == 0);
        mpc.subject_to(mpc.at_tf(x2) == 0);


Set solver back-end

.. tab:: Python

    .. code:: python

        mpc.solver('ipopt')


.. tab:: MATLAB

    .. code:: matlab

        mpc.solver('ipopt');

Set transcription method

.. tab:: Python

    .. code:: python

        method = MultipleShooting(N=40)
        mpc.method(method)


.. tab:: MATLAB

    .. code:: matlab

        method = MultipleShooting('N',40);
        mpc.method(method);

Solve the underlying optimal control problem

.. tab:: Python

    .. code:: python

        mpc.solve()


.. tab:: MATLAB

    .. code:: matlab

        mpc.solve();



********************************
Post-processing the MPC solution
********************************

.. todo::
    How to post-process the solution of the MPC solver (sampler, gist, etc.).

***************************************************
Generating artifacts for prototyping and deployment
***************************************************

How to export artifacts in Impact.

.. tab:: Python

    .. code:: python

        mpc.export('double_integrator')

.. tab:: MATLAB

    .. code:: matlab

        mpc.export('double_integrator');
