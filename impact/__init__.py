from .mpc import MPC
from .model import Model
from .dotdict import DotDict
from .helpers import simulink_parse_states
from rockit import DirectCollocation, MultipleShooting, SingleShooting, matlab_path, external_method
import os

try:
  matlab_path = matlab_path+os.pathsep+os.path.realpath(os.path.dirname(os.path.realpath(__file__)))
except:
  matlab_path = "not_found"

build_dir_suffix = "_build_dir"
build_dir_prefix = ""