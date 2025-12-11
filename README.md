# Impact: A Toolchain for Nonlinear Model Predictive Control Specification, Prototyping, and Deployment

<div align="center">

[![pipeline status](https://gitlab.kuleuven.be/meco-software/impact/badges/main/pipeline.svg)](https://gitlab.mech.kuleuven.be/meco-software/impact/commits/main)
[![license: LGPL v3](https://img.shields.io/badge/license-LGPL%20v3-success.svg)](https://opensource.org/licenses/LGPL-3.0)
[![pypi version](https://badge.fury.io/py/impact-meco.svg)](https://pypi.org/project/impact-meco/)
[![arXiv](https://img.shields.io/badge/math.OC-2303.08850-b31b1b?logo=arxiv&logoColor=red)](https://arxiv.org/abs/2303.08850)
[![html docs](https://img.shields.io/static/v1.svg?label=docs&message=online&color=informational)](https://meco-software.pages.gitlab.kuleuven.be/impact/)
</div>

<div align="center">

<a href="https://gitlab.kuleuven.be/meco-software/impact"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
<a href="https://gitlab.kuleuven.be/meco-software/impact"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a>

</div>


## Description

Impact is a flexible toolchain for nonlinear model predictive control (NMPC) specification with automatic code generation capabilities. 

The toolchain reduces the engineering complexity of NMPC implementations by providing the user with an easy-to-use application programming interface, and with the flexibility of using multiple state-of-the-art tools and numerical optimization solvers for rapid prototyping of NMPC solutions. 

Impact is written in Python, users can call it from Python and MATLAB, and the generated NMPC solvers can be directly executed from C, Python, MATLAB and Simulink.

**License:** Impact is released under the [GNU LGPLv3 license](LICENSE).

## Installation

### Option 1: Installing with pip
You can install Impact (ideally into a virtual environment) via pip using the following command:

```
pip install impact-meco
```

### Option 2: Installing from cloned repository
Alternatively, you can clone this repository and install Impact from source. You just need to (i) clone the repository, (ii) move into Impact's root directory, and (iii) run the `setup.py` script with the `install` option. It will install your application into the virtualenv site-packages folder and also download and install all dependencies:

```
git clone https://gitlab.kuleuven.be/meco-software/impact.git
cd impact
python setup.py install
```
You could also use the `develop` option, instead of `install`, during the execution of `setup.py` as `python setup.py develop`. 
This has the advantage of just installing a link to the site-packages folder instead of copying the data over. You can then modify/update the source code without having to run `python setup.py install` again after every change.


## Submitting an issue

Please submit an issue if you want to report a bug or propose new features of the toolchain.

## Citing

If you use this library, we would be grateful if you could cite [the following paper](https://doi.org/10.1016/j.ifacol.2023.10.1451): 
<details>
<summary> A. Florez, A. Astudillo, W. Decré, J. Swevers, and J. Gillis, "IMPACT: A Toolchain for Nonlinear Model Predictive Control Specification, Prototyping, and Deployment", IFAC-PapersOnLine, vol. 56, no. 2, pp. 3164–3169, 2023,
doi: 10.1016/j.ifacol.2023.10.1451
</summary>
<p>

```
@article{Florez2023,
  title = {IMPACT: A Toolchain for Nonlinear Model Predictive Control Specification, Prototyping, and Deployment},
  author = {Alvaro Florez and Alejandro Astudillo and Wilm Decré and Jan Swevers and Joris Gillis},
  journal = {IFAC-PapersOnLine},
  volume = {56},
  number = {2},
  pages = {3164-3169},
  year = {2023},
  note = {22nd {IFAC} World Congress},
  issn = {2405-8963},
  doi = {10.1016/j.ifacol.2023.10.1451},
  url = {https://www.sciencedirect.com/science/article/pii/S2405896323018591}
}
```
</p>
</details>

<br/>

If you use the ROS 2 package generator of Impact, we would appreciate if you cite [the following paper](http://dx.doi.org/10.1109/AMC58169.2024.10505632):
<details>
<summary> A. Astudillo, A. Florez, W. Decré, and J. Swevers, "Rapid Deployment of Model Predictive Control for Robotic Systems: From IMPACT to ROS 2 Through Code Generation", in Proceedings of the 2024 IEEE 18th International Conference on Advanced Motion Control (AMC), 2024,
doi: 10.1109/amc58169.2024.10505632
</summary>
<p>

```
@inproceedings{AstudilloAMC2024,
  title = {Rapid Deployment of Model Predictive Control for Robotic Systems: From IMPACT to ROS 2 Through Code Generation},
  author = {Alejandro Astudillo and Alvaro Florez and Wilm Decré and Jan Swevers},
  url = {http://dx.doi.org/10.1109/AMC58169.2024.10505632},
  doi = {10.1109/amc58169.2024.10505632},
  booktitle = {2024 {IEEE} 18th International Conference on Advanced Motion Control (AMC)},
  publisher = {IEEE},
  year = {2024},
  month = feb 
}
```
</p>
</details>

