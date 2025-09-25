*************
Installation
*************

The Impact toolchain is compatible with both Windows and Linux operating systems. For optimal performance, we recommend installing it within a dedicated virtual environment. This ensures smoother package management and helps prevent collisions with other software.

You can use an environment manager such as `Conda <https://conda.io/projects/conda/en/latest/index.html>`_ or an easy package and environment manager like `Anaconda <https://www.anaconda.com/>`_ to create the environment. 
In this environment, you will install the Impact packages and launch either Matlab or Python from it.   

**Note:** If you intend to use Matlab, remember to select a Python version compatible with your Matlab version while creating the virtual environment. Compatibility details can be found `here <https://nl.mathworks.com/support/requirements/python-compatibility.html?s_tid=srchtitle_site_search_1_python%20compatibility>`_ 

**Option 1**

You can install Impact via PyPi by using

    .. code:: bash

        pip install impact-meco


**Option 2**

You can clone the `repository <https://gitlab.kuleuven.be/meco-software/impact>`_  by calling

    .. code:: bash

        git clone git@gitlab.kuleuven.be:meco-software/impact.git

To install Impact just run the setup.py script with the install parameter. It will install your application into the virtualenv site-packages folder and also download and install all dependencies:

    .. code:: python

        python setup.py install


If you are developing on Impact and also want the requirements to be installed, you can use the develop command instead:

    .. code:: python

        python setup.py develop (deprecated)
        pip install -e .

This has the advantage of just installing a link to the site-packages folder instead of copying the data over. You can then continue to work on the code without having to run install again after each change.



**Aditional step needed for Matlab**

The Impact toolchain relies on Rockit and Casadi. These dependencies are necessary during installation. To integrate Casadi with Matlab, an additional step is required.

1. Download the appropriate Casadi release from the `Casadi <https://github.com/casadi/casadi/releases>`_ repository. Ensure it matches the installed version (execute :code:`pip list` in your environment's terminal to confirm the installed Casadi version).
2. Unzip the downloaded file and add it to the Matlab path (without subdirectories).
3. Remove any other Casadi versions from the Matlab path to avoid conflicts.