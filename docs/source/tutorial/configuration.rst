.. role:: raw-math(raw)
    :format: latex html

.. role:: matlab(code)
   :language: matlab
   

The configuration options in Impact are used to define the system model, cost function, constraints, the transcription method and the solver. The solver is the tool that solves the optimal control problem (OCP) at each sample time, while the transcription method is the procedure to transform the OCP to a nonlinear programming (NLP) problem.


**********************
MPC object
**********************
To start the MPC object should be created using Impact library calling the method :code:`mpc = MPC()` (Based OCP object of Rockit API `.MPC <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.ocp.Ocp>`_). The MPC object is the main interface for defining the OCP. 
It provides methods to define the system model, cost function, constraints, and transcription method.
The MPC object is created with the following parameters:

* :code:`t0` is the starting time of the optimal control problem (OCP) horizon. It is a scalar value in seconds. The default value is 0.
* :code:`T` is the total horizon time of the OCP. It can be fixed or free. If it is fixed, it is a scalar value in seconds. If it is free, it is a symbolic variable. The default value is 1.
  
  An example for fixed and free horizon is shown in :ref:`Instantating the MPC object` at the getting started



**********************
Model
**********************

The model can be defined straight forward describing explicitly the ODEs equations as shown in Getting started :ref:`Defining a system model`. 
But it can also be defined using a YAML file. The YAML file can contain the ODEs equations, or integrate external functions produced by third-party software. 

------------------------
YAML file using ODE  
------------------------
The YAML file is a plaintext document containing the system model description. Using the method :code:`.add_model`, you can integrate the YAML file model into the problem.

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

**Note:** This description should use Python notation


-------------------------------
YAML file with external ODEs
-------------------------------
YAML file can integrate external functions produced by third-party software representing the system.

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




---------------------------
Discrete Time Model
---------------------------
Let's assume we have a system model in discrete time. For example,
the following difference equations of a double integrator describe the system

:math:`\begin{equation}
\begin{aligned}
x_1(k+1) & = x_1(k)+ 0.5*x_2(k) \\
x_2(k+1) & = u(k)
\end{aligned}
\end{equation}`

.. tab:: Python

    .. code:: python

        x1 = mpc.state()
        x2 = mpc.state()

        u = mpc.control()

        mpc.set_next(x1, x1+0.5*x2)
        mpc.set_next(x2, u)

.. tab:: MATLAB

    .. code:: matlab

        x1 = mpc.state();
        x2 = mpc.state();

        u = mpc.control();

        mpc.set_next(x1, x1+0.5*x2);
        mpc.set_next(x2, u);

The method :code:`.set_next` allows us to define derivatives for all state variables, the evolution each sampling time.


************************
Parameters
************************
The parameters are defined using the method :code:`.parameter()`  (Based Rockit API `.parameter <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=param#rockit.stage.Stage.parameter>`_). Name and dimension are the values required. The parameters can be defined as a scalar, vector, or matrix. The parameters can be used to define the system model, cost function, and constraints. 

The user should define the parameters values using the method :code:`.set_value()` (Based Rockit API `set_value <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.set_value>`_). The parameters values can be set as a scalar, vector, or matrix depending on how they were defined. This values can be updated at each sample time.

Here we consider optimal control problems (OCP) including the system model, it means, the initial state of the system is always required. A parameter with the initial state is required and it should **always** has the name *'x_current'*. 

.. tab:: Python

    .. code:: python

        x_current = mpc.parameter('x_current', 4)
        mpc.set_value(x_current, [0, 0, 0, 0])

.. tab:: MATLAB

    .. code:: matlab

        x_current = mpc.parameter('x_current', 4);
        mpc.set_value(x_current, [0; 0; 0; 0]);


************************
Expression specifiers
************************
There are several ways to specify or modify expressions in Impact.

* :code:`.at_t0` and :code:`.at_tf` methods are used to define the expression at the initial and final sample of the horizon, respectively. They can be used to define the cost function, and constraints. (Based Rockit API `.at_t0 <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.at_t0>`_ , `.at_tf <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.at_tf>`_)
* :code:`.prev` and :code:`.next` methods are used to define the expression at the previous and next sample of the horizon, respectively (Based Rockit API `.prev <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.prev>`_ , `.next <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.next>`_). They can be used to define the cost function, and constraints. Specially useful for defining limits in the change of the control or states variables. 
    :code:`ocp.next(u)-u` defines the difference between the next control and the current control.


************************
Cost function
************************

The cost function is defined using the method :code:`.add_objective`. 
The cost function can be define as any expression in terms of the states, controls, and parameters.
Usually this expression is considered for the horizon of the problem, for this reason, there are two options to define the expression within the horizon

* :code:`.integral()` (Based Rockit API `.integral <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.integral>`_) method, which definesthe integral of an expression for the whole horizon. Additionally, grid options can be set to define the integration points.
    *  grid='inf' 
    *  grid='control' 

    The final stage is not included in this definition. 
* :code:`.sum()` (Based Rockit API `.sum <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.sum>`_) method, which defines sum of the expression for each control point within the horizon. Only grid 'control' is available. 'include_last' option can be set to include the final sample.

.. note::
    If the system model is define as discrete time, the cost function should be define as sum.


************************
Constraints
************************

The method :code:`.subject_to()` is used to define constraints in the OCP (Based Rockit API `.subject_to <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.subject_to>`_).
The constraints can be defined as any expression in terms of the states, controls, and parameters.
It has a argument the constrained expression. It should be a symbolic expression that depends on decision variables and features a comparison using `==`, `<=`, `=>`.
It is considered over the entire horizon. Expression specifiers can be used to specify the contraints to certain samples in within the horizon.

Additional options can be set to define the :code:`'grid'` of the constraints, also the :code:`include_first` and :code:`include_last` options can be set to include the first and last sample of the horizon.
Check the API documentation for more details on the options available.

.. note::
    Be aware that in Matlab the multiple comparison in a single line does not require double parenthesis, but it Python it does. See the example of getting started. 
    :ref:`Set_constraints`


************************
Transcription method
************************

The method :code:`.method()` defines the transcription method, it converts the problem into a Nonlinear Programming (NLP) problem.
The following methods are available:

* :code:`SingleShooting()` (Based Rockit API and CasADi `SingleShooting_Method <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=single#rockit.single_shooting.SingleShooting>`_ and `SingleShooting_CasADi <https://web.casadi.org/docs/#direct-single-shooting>`_), Single shooting is a method where the entire trajectory of the system is parameterized, and the optimization problem is solved for all time steps at once. 
* :code:`MultipleShooting()` (Based Rockit API and CasADi `.MultipleShooting_Method <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=single#rockit.multiple_shooting.MultipleShooting>`_ and `.MultipleShooting_CasADi <https://web.casadi.org/docs/#direct-multiple-shooting>`_ ) In multiple shooting, the trajectory is divided into smaller segments, and the optimization problem is solved for each segment with continuity stitching constraints. This method can provide better numerical stability compared to single shooting.
* :code:`DirectCollocation()` (Based Rockit API and CasADi `DirectCollocation_Method <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=single#rockit.direct_collocation.DirectCollocation>`_ and `DirectCollocation_CasADi <https://web.casadi.org/docs/#direct-collocation>`_ )  This is a method where the trajectory is approximated using polynomial functions, and the optimization problem is solved by enforcing constraints at specific collocation points. 

  There are different options to configure the transcription method. The most important are the following:

  * :code:`N` and :code:`M` are the number of control points and the number of segments, respectively. The default values are 50 and 1, respectively.
  * :code:`intg` is the integration method used to compute the dynamics of the system. The default value is RungeKutta four :code:`rk`. Other option is explicit euler :code:`expl_euler`. These two are inherited from Rockit, they allow to make use of signal sampling with 'refine'.
  
    Different integrators are implemnted as plugins in casadi, they can be consuled in `integrator <https://web.casadi.org/api/internal/dd/d1b/group__integrator.html>`_.

  * :code:`grid` is the grid used for the integration. Options are:

    * :code:`UniformGrid()` This option defines a grid with uniform spacing.  `UniformGrid <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=multipleshooting#rockit.sampling_method.UniformGrid>`_ 
    * :code:`FreeGrid()` This option defines a grid with unprescribed spacing.  `FreeGrid <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.sampling_method.FreeGrid>`_ 
    * :code:`DensityGrid()` The grid is defined based on one symbolic variable expression (dimensionless time) that describes the density of the grid
    * :code:`GeometricGrid()`  This grid grow geometrically each sampling time.  `GeometricGrid <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=freegrid#rockit.sampling_method.GeometricGrid>`_ 


.. note::
    If the horizon time is fixed, in combination with the number of control points they define the sampling time.
    If the horizon time is not fixed (FreeTime), the sampling time is defined by the number of control points and the horizon time solution.    



************************
Solver
************************
The solver is the most important part of the OCP. It is the tool that solves the optimization problem at each sample time. Impact allows to instantiate the solvers interfaced in Rockit and CasADi. 
The solver is defined using the method :code:`.solver()`. Each solver has its own options and parameters. The following solvers are available:

    * :code:`sqpmethod` standard definition of a SQP method. `sqpmethod <https://web.casadi.org/python-api/#sqpmethod>`_ For this method, an inner QP solver is required, it should be set in the option :code:`qpsol`. 
        More common options are:

        * :code:`qrqp` `qrqp <https://web.casadi.org/python-api/#qrqp>`_ This method solves QP problems using an active-set method.
        * :code:`osqp` `osqp_CasADi <https://web.casadi.org/python-api/#osqp>`_, `osqp <https://osqp.org/docs/>`_  This method solves convex quadratic programs (QP) problems using an operator splitting method. 
        * :code:`qpoases` `qpoases <https://web.casadi.org/python-api/#qpoases>`_ is a solver using an active Set Strategy.

        For more options check the CasADi documentation of QP solvers `qp <https://web.casadi.org/python-api/#qp>`_.
  
    * :code:`Ipopt` `Ipopt_CasADi <https://web.casadi.org/python-api/#ipopt>`_, `Ipopt <https://coin-or.github.io/Ipopt/>`_ is a well konwn open-source solver for solving large-scale nonlinear optimization problems. It is based on the primal-dual interior point method and uses a line search strategy to handle constraints. 
    * :code:`Fatrop` `Fatrop <https://web.casadi.org/python-api/#fatrop>`_ is a fast and robust solver (Ipopt inspired) for nonlinear programming problems. It is fast by exploiting the optimal control problem structure through a specialized linear solver.


:code:`Acados` `Acados <https://docs.acados.org/>`_ is a toolbox for optimal control and nonlinear MPC, it offers QP, and NLP solvers tailored for real-time applications. It interfaces different solvers like HPIPM, qpOASES, DAQP and OSQP.
Acados can be used in Impact, it is invoked using the function :code:`external_method()` in the :code:`.method()` definition of the problem. It defines transcription and solver, all acados parameters and solvers are configured there.

----------------------
Initial guess
----------------------

The method :code:`set_initial` refers to the initial guess of the optimization problem and **not** the initial state of the system.

The initial guess is used to initialize the optimization algorithm, and it can be set to a specific value (constant or vector within the horizon) or left it as zero default.
`Set_initial <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html#rockit.stage.Stage.set_initial>`_   


Once the problem is defined, it can be solved using the method :code:`.solve()`. 
Also, the problem can be exported directly to generate the Impact artifacts.





************************
Retrieve the solution
************************
After solving the problem, the Impact toolchain provides multiple ways to access the solution, allowing the user to extract variables at specific grid points. 
The solution is retrieved using the method :code:`.sample()` (Based Rockit API `sample <https://meco-software.pages.gitlab.kuleuven.be/rockit/apidoc/rockit.html?highlight=sample#rockit.stage.Stage.sample>`_)
As argument the user should define the variable or expression to be retrieved. The variable can be a state, control, or parameter. The expression can be any expression in terms of the states, controls, and parameters.
Also a grid option should be set. The grid option defines the points in the horizon where the solution is retrieved. The grid options are:

* :code:`grid='control'` The solution is retrieved at the control points of the grid.
* :code:`grid='integrator'` The solution is retrieved at the integral points of the grid.
* :code:`grid='integrator_roots'` The solution is retrieved at the roots of the integrator. This option is only available for the :code:`DirectCollocation()` method.

:code:`refine` is an additional (optional) argument. It defines the number of points to be retrieved between the integration points. The default value is 1.   

Following figure shows a zoom-in at the beginning of the position of the example of getting started from :ref:`Post-processing_the_MPC_solution`.
Let's remember that the OCP is defined in a fixed horizon, and the sampling time is defined by the number of control points and the horizon time.

* The control grid is define by the number of control points :code:`N=50` and the horizon :code:`2s`, therefore,the sampling time is :code:`0.04` seconds.
* For the case of :code:`M=2`, the integrator grid is defined as two segments between control points, then, the sampling time is :code:`0.02` seconds.
* If additionally to :code:`M=2`, the refine option is set to :code:`5`, then the grid is defined as :code:`5` segments between integration points, then, the sampling time is :code:`0.004` seconds.

.. tab:: Python

    .. image:: imagesGetting/posPyZoom.png
        :width: 350 px
        :height: 300 px
        :align: center

.. tab:: MATLAB

    .. image:: imagesGetting/posMatZoom.png
        :width: 350 px
        :height: 300 px
        :align: center



************************
Exportation
************************
This is the main capability of Impact. The toolchain allows to export the problem with different configurations.
The method :code:`.export()` make the exportation and create a folder with the name: *export name + _build_dir*. The folder will contain all the files and components of the different artifacts.
The exportation options are:

  *  :code:`name` is the name of the exportation. It is used to create the folder where the artifacts are stored.
  *  :code:`src_dir` is the source directory where the artifacts are stored. The default value is `.` which is the current working directory.
  *  :code:`compile` is a boolean value that indicates whether to compile the artifacts or not. The default value is `True`.
  *  :code:`use_codegen` is a boolean value that indicates whether to use code generation or not. The default value is `True`. If it is set to `False`, the artifacts are generated without code generation. 
  *  :code:`short_output` is a boolean value, with default value `True`.
  
    * :code:`True` the output is short. The outputs are states at next sampling time (k=1), and the controls at the current sampling time (k=0).
    * :code:`False` the output is long. The outputs are states and controls for the whole horizon.
  
  *  :code:`ignore_errors` is a boolean value that indicates whether to ignore errors or not while solving the optimization problem in simulation. The default value is `False`. 
  *  :code:`qp_error_on_fail` is a boolean value that indicates whether to raise an error if the QP solver fails or not in case using a solver that involve QP solutions. The default value is `True`. 
  *  :code:`context` allow to specify the environment :code:`"matlab"`, default value is :code:`None`.  
  *  :code:`ros2` is a boolean value that indicates whether to export the ROS2 node or not. The default value is `False`.
  *  :code:`ros2_options` is a ditionary with the options for the ROS2 node. The options are:
  
    * :code:`'repeat_on_fail'` is a string that indicates whether to repeat the excecution on failure or not. 

The exportation creates artifacts for the MPC controller, and additional functions for the cost function and for the discrete version of the system model. They are useful for simulation and fot prototyping purposes.  

