*************
Installation
*************

**Option 1**

You can install Impact via PyPi by using

    .. code:: bash

        pip install impact-meco


**Option 2**

You can clone the `repository <https://gitlab.kuleuven.be/meco-software/impact>`_  by calling

    .. code:: bash

        git clone git@gitlab.kuleuven.be:meco-software/impact.git

To install Impact (ideally into a virtualenv) just run the setup.py script with the install parameter. It will install your application into the virtualenv site-packages folder and also download and install all dependencies:

    .. code:: python

        python setup.py install


If you are developing on Impact and also want the requirements to be installed, you can use the develop command instead:

    .. code:: python

        python setup.py develop

This has the advantage of just installing a link to the site-packages folder instead of copying the data over. You can then continue to work on the code without having to run install again after each change.