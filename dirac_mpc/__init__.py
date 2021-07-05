from .mpc import MPC
from .impact import Impact
from rockit import DirectCollocation, MultipleShooting, SingleShooting, matlab_path
import os

try:
  matlab_path = matlab_path+os.pathsep+os.path.realpath(os.path.dirname(os.path.realpath(__file__)))
except:
  matlab_path = "not_found"

build_dir_suffix = "_build_dir"
build_dir_prefix = ""