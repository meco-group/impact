from casadi import veccat, MX, vcat
import copy

keywords = {"x","u","z","p","c","y"}
for k in set(keywords):
  keywords.add("n"+k)
keywords.add("all")
keywords.add("ode")

class DotDict(object):
  """like a dict but access through . """
  def __init__(self, d=None):
    if d is None:
      d = {}
    self._d = d

  def __getattr__(self, k):
    try:
      return self._d[k]
    except:
      raise AttributeError()

  def __deepcopy__(self, memo):
    if id(self) in memo:
        return memo[id(self)]
    ret = self.__class__(copy.deepcopy(self._d))
    
    memo[id(self)] = ret
    return ret

  def _update(self, d, allow_keyword=False):
    for k,v in d.items():
      if allow_keyword:
        if k in self._d:
          old = self._d[k]
          if isinstance(old,list):
            self._d[k]=old+v
          else:
            self._d[k] = veccat(old,v)
        else:
          self._d[k] = v
      else:
        if k in keywords:
          raise Exception("'%s' is a reserved keyword. Please use another name." % k)
        if k in self._d:
          raise Exception("Name collision: '%s' already in use." % k)
        self._d[k] = v
      
  def __repr__(self,indent=0):
    s = "{\n"
    for k,v in sorted(self._d.items()):
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
  def __init__(self, definition,prefix=""):
    if isinstance(definition,MX):
      self._symbols = [definition]
      self._concat = definition
      self._numel = definition
      self._names = [definition.name()]
    else:
      symbols = []
      for d in definition:
        size = d["size"] if "size" in d else 1
        name = d["name"]
        symbols.append(MX.sym(prefix+name, size))
      self._names = [d["name"] for d in definition]
      self._symbols = symbols
      self._concat = vcat(symbols)
      self._numel = self._concat.numel()

  def __MX__(self):
    return self._concat


def remove_prefix(n,prefix):
  if n.startswith(prefix):
    return n[len(prefix):]
  else:
    n