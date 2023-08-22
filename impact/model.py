from casadi import MX, vvcat
from .dotdict import DotDict, Structure, remove_prefix

class Model(DotDict):
  """This should be a description of the Model class."""

  def __init__(self, prefix=""):
    DotDict.__init__(self)
    self._prefix = prefix
    self.all = DotDict()

  def _register(self,key,parts):
    if isinstance(parts,dict):
      for k,v in parts.items():
        self._update({k: v})
      ks = list(sorted(parts.keys()))
      self.all._update({key: [parts[k] for k in ks]},allow_keyword=True)
      self._update({key: vvcat([parts[k] for k in ks])},allow_keyword=True)
    else:
      s = Structure(parts,prefix=self._prefix)
      for e in s._symbols:
        self._update({remove_prefix(e.name(),self._prefix): e},allow_keyword=isinstance(parts,MX))
      self.all._update({key: s._symbols},allow_keyword=True)
      if not isinstance(parts,MX):
        self._update({key : s._concat},allow_keyword=True)
      self._update({'n'+key: s._numel},allow_keyword=True)
      return s