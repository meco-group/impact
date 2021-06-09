from rockit import Ocp
from casadi import Function, MX, vcat, vvcat, veccat
import casadi
import yaml
import os


dae_keys = {"x": "differential_states", "z": "algebraic_states", "p": "parameters", "u": "controls"}
dae_rockit = {"x": "state", "z": "algebraic", "p": "parameter", "u": "control"}


keywords = {"x","u","z","p","c","y"}
for k in set(keywords):
  keywords.add("n"+k)
keywords.add("all")

class DotDict:
  """like a dict but access through . """
  def __init__(self, d={}):
    for k,v in d.items():
      setattr(self,k,v)
  
  def _update(self, d, allow_keyword=False):
    for k,v in d.items():
      if allow_keyword:
        if hasattr(self,k):
          old = getattr(self,k)
          if isinstance(old,list):
            setattr(self,k,old+v)
          else:
            setattr(self,k,veccat(old,v))
        else:
          setattr(self,k,v)
      else:
        if k in keywords:
          raise Exception("'%s' is a reserved keyword. Please use another name." % k)
        if hasattr(self, k):
          raise Exception("Name collision: '%s' already in use." % k)
        setattr(self,k,v)
      
  def __repr__(self,indent=0):
    s = "{\n"
    for k,v in sorted(self.__dict__.items()):
      s += ("  " * (indent+1)) + k
      try:
        s += ": " + v.__repr__(indent=indent+1)
      except:
        if isinstance(v, list):
          s+= ": " + str(v)
        #s += str(v)
        pass

      s += "\n"
    s+=("  " * indent) + "}"
    return s

class Structure:
  def __init__(self, definition):
    if isinstance(definition,MX):
      self._symbols = [definition]
      self._concat = definition
      self._numel = definition
    else:
      symbols = []
      for d in definition:
        size = d["size"] if "size" in d else 1
        name = d["name"]
        symbols.append(MX.sym(name, size))
      self._symbols = symbols
      self._concat = vcat(symbols)
      self._numel = self._concat.numel()

  def __MX__(self):
    return self._concat

class Model(DotDict):
  def __init__(self):
    self.all = DotDict()

  def _register(self,key,parts):
    if isinstance(parts,dict):
      for k,v in parts.items():
        self._update({k: v})
      ks = list(sorted(parts.keys()))
      self.all._update({key: [parts[k] for k in ks]},allow_keyword=True)
      self._update({key: vvcat([parts[k] for k in ks])},allow_keyword=True)
    else:
      s = Structure(parts)
      for e in s._symbols:
        self._update({e.name(): e},allow_keyword=isinstance(parts,MX))
      self.all._update({key: s._symbols},allow_keyword=True)
      if not isinstance(parts,MX):
        self._update({key : s._concat},allow_keyword=True)
      self._update({'n'+key: s._numel},allow_keyword=True)
      return s._concat

class MPC(Ocp):
  def __init__(self,*args,**kwargs):
    Ocp.__init__(self, *args, **kwargs)
    self.expr = Model()

  def parameter(self,name,*args,**kwargs):
    p = MX.sym(name,*args)
    self.register_parameter(p,**kwargs)
    self.expr._register('p', {name: p})
    return p

  def add_model(self,name,file_name):
    # Read meta information
    with open(file_name) as file:
        model_meta = yaml.load(file, Loader=yaml.FullLoader)

    # Read equations
    if not "equations" in model_meta:
      raise Exception("Model file is missing 'equations' section.")
    assert model_meta["equations"]["type"]=="casadi_serialized"
    model_file_name = os.path.expanduser(model_meta["equations"]["file_name"])
    if not os.path.isabs(model_file_name):
      model_file_name = os.path.join(os.path.dirname(file_name),model_file_name)
    model = Function.load(model_file_name)
    # Make sure the CasADi Function adheres to a standard
    expected_names = dae_keys.keys()
    unknowns = set(model.name_in()).difference(set(expected_names))
    if unknowns:
      raise Exception("These input names are not understood: %s. Use the following canonical names: %s." % (unknowns, expected_names))
    
    for i in range(model.n_in()):
      assert model.sparsity_in(i).is_vector()

    
    m = Model()

    args = {}
    nd = {}
    # Get dimensions
    for key, long_name in dae_keys.items():
      rockit_name = dae_rockit[key]
      n = model.numel_in(key) if key in model.name_in() else 0
      nd[key] = n
      if n:
        if long_name in model_meta:
          m._register(key,model_meta[long_name])
          v = getattr(m,key)
          assert v.numel()==n
          register = getattr(self,"register_"+rockit_name)
          for e in v.primitives():
            register(e)
        else:
          v = getattr(self,rockit_name)(n)
          m._register(key, v)
        args[key] = v

    res = model(**args)
    if nd["x"]:
      assert "ode" in res
      self.set_der(m.x, res["ode"])
    if nd["z"]:
      assert "alg" in res
      self.add_alg(res["alg"])

    # Define constants
    constants = {}
    definitions = casadi.__dict__
    if "constants" in model_meta:
      if "inline" in model_meta["constants"]:
        for k,v in model_meta["constants"]["inline"].items():
          try:
            constants[k] = eval(v,casadi.__dict__,m.__dict__)
          except:
            constants[k] = v
    m._register("c", constants)

    # Define Outputs
    outputs = {}
    definitions = casadi.__dict__
    if "outputs" in model_meta:
      if "inline" in model_meta["outputs"]:
        for k,v in model_meta["outputs"]["inline"].items():
          outputs[k] = eval(v,casadi.__dict__,m.__dict__)
    m._register("y", outputs)


    self.expr._update({name: m})
    return m

