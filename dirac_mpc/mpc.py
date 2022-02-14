from rockit import Ocp
from casadi import Function, MX, vcat, vvcat, veccat, GlobalOptions, vec, CodeGenerator
import casadi
import yaml
import os
from collections import OrderedDict
import shutil
from zipfile import ZipFile 
from lxml import etree
from contextlib import redirect_stdout, redirect_stderr
import io
import numpy as np

dae_keys = {"x": "differential_states", "z": "algebraic_states", "p": "parameters", "u": "controls"}
dae_rockit = {"x": "state", "z": "algebraic", "p": "parameter", "u": "control"}


keywords = {"x","u","z","p","c","y"}
for k in set(keywords):
  keywords.add("n"+k)
keywords.add("all")


def remove_prefix(n,prefix):
  if n.startswith(prefix):
    return n[len(prefix):]
  else:
    n

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


class Model(DotDict):
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

def fun2s_function(fun, name=None, dir=".",ignore_errors=False):
    fun_name = fun.name()
    if name is None:
      name = fun_name
    s_function_name = name+"_s_function_level2"
    cg_options = {"casadi_real": "real_T", "casadi_int": "int_T"}
    cg = CodeGenerator(name, cg_options)
    cg.add(fun)
    code = cg.dump()
    for i in range(fun.n_in()):
      if not fun.sparsity_in(i).is_dense():
        raise Exception("Sparse inputs not supported")
    for i in range(fun.n_out()):
      if not fun.sparsity_out(i).is_dense():
        raise Exception("Sparse outputs not supported")

    s_function_file_name_base = s_function_name+".c"
    s_function_file_name = os.path.join(dir,s_function_file_name_base)

    with open(s_function_file_name,"w") as out:
      out.write(f"""
        #define S_FUNCTION_NAME {s_function_name}
        #define S_FUNCTION_LEVEL 2

        #include "simstruc.h"
        {code}

        static int_T n_in, n_out;
        static int_T sz_arg, sz_res, sz_iw, sz_w;

        static void mdlInitializeSizes(SimStruct *S) {{
          ssSetNumSFcnParams(S, 0);
          if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {{
              return; /* Parameter mismatch will be reported by Simulink */
          }}

        /* Read in CasADi function dimensions */
        n_in  = {fun_name}_n_in();
        n_out = {fun_name}_n_out();
        {fun_name}_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
        
        /* Set up simulink input/output ports */
        int_T i;
        if (!ssSetNumInputPorts(S, n_in)) return;
        for (i=0;i<n_in;++i) {{
          const int_T* sp = {fun_name}_sparsity_in(i);
          /* Dense inputs assumed here */
          ssSetInputPortDirectFeedThrough(S, i, 1);
          ssSetInputPortMatrixDimensions(S, i, sp[0], sp[1]);
          ssSetInputPortRequiredContiguous(S, i, 1);
        }}

        if (!ssSetNumOutputPorts(S, n_out)) return;
        for (i=0;i<n_out;++i) {{
          const int_T* sp = {fun_name}_sparsity_out(i);
          /* Dense outputs assumed here */
          ssSetOutputPortMatrixDimensions(S, i, sp[0], sp[1]);
        }}

        ssSetNumSampleTimes(S, 1);
        
        /* Set up CasADi function work vector sizes */
        ssSetNumRWork(S, sz_w);
        ssSetNumIWork(S, sz_iw);
        ssSetNumPWork(S, sz_arg+sz_res);
        ssSetNumNonsampledZCs(S, 0);

        /* specify the sim state compliance to be same as a built-in block */
        ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

        ssSetOptions(S,
                    SS_OPTION_EXCEPTION_FREE_CODE |
                    SS_OPTION_USE_TLC_WITH_ACCELERATOR);

        /* Signal that we want to use the CasADi Function */
        {fun_name}_incref();
        }}

        static void mdlInitializeSampleTimes(SimStruct *S)
        {{
            ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
            ssSetOffsetTime(S, 0, 0.0);
            ssSetModelReferenceSampleTimeDefaultInheritance(S); 
        }}

        static void mdlOutputs(SimStruct *S, int_T tid)
        {{

          /* Set up CasADi function work vectors */
          void** p = ssGetPWork(S);
          const real_T** arg = (const real_T**) p;
          p += sz_arg;
          real_T** res = (real_T**) p;
          real_T* w = ssGetRWork(S);
          int_T* iw = ssGetIWork(S);
          
          
          /* Point to input and output buffers */
          int_T i;   
          for (i=0; i<n_in;++i) {{
            arg[i] = ssGetInputPortSignal(S,i);
          }}
          for (i=0; i<n_out;++i) {{
            res[i] = ssGetOutputPortRealSignal(S,i);
          }}

          /* Get a hold on a location to read/write persistant internal memory
          */

          int mem = {fun_name}_checkout();

          /* Run the CasADi function */
          int_T ret = {fun_name}(arg,res,iw,w,mem);

          if (ret && {int(not ignore_errors)}) {{
              static char msg[{100+len(s_function_name)}];
              sprintf(msg, "SFunction '{s_function_name}' failed to compute (error code %d) at t=%.6fs.", ret, ssGetT(S));
              ssSetLocalErrorStatus(S, msg);
          }}

          /* Release hold */
          {fun_name}_release(mem);

        }}

        static void mdlTerminate(SimStruct *S) {{
          {fun_name}_decref();
        }}


        #ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
        #include "simulink.c"      /* MEX-file interface mechanism */
        #else
        #include "cg_sfun.h"       /* Code generation registration function */
        #endif
        """)
    return s_function_file_name_base


class Diagram:
  def __init__(self,template,simulink_library_name,simulink_library_dirname=None):
    self.masks = []
    self.template = template
    self.simulink_library_name = simulink_library_name
    self.simulink_library_dirname = simulink_library_dirname
    self.simulink_library_filename = simulink_library_dirname+".slx"
    self.next_id = 1

  def add(self,mask):
    mask.register(self.next_id)
    self.next_id = mask.max_id
    self.masks.append(mask)

  def write(self):
    with ZipFile(self.template) as zipfile:
      zipfile.extractall(path=self.simulink_library_dirname)
    blockdiagram_filename = os.path.join(self.simulink_library_dirname,"simulink","blockdiagram.xml")
    with open(blockdiagram_filename,'r') as blockdiagram_file:
      tree = etree.parse(blockdiagram_file)

    system = tree.find('.//System')
    for e in system.findall('Block'):
      system.remove(e)
    for e in system.findall('Line'):
      system.remove(e)


    height_spacing = 100
    max_width = 1920
    offset_x = 0
    offset_y = 0
    height = 0
    max_id = 0
    for m in self.masks:
      # Overflow
      if offset_x+m.width>max_width and offset_x>0:
        offset_y += height+height_spacing
        offset_x = 0
      else:
        offset_x += m.width
        height = max(height, m.height)
      m.write(system,offset_left = offset_x, offset_top=offset_y)
      max_id = max(max_id, m.max_id)

    highwater = system.find('P[@Name="SIDHighWatermark"]')
    highwater.text = str(max_id)

    tree.write(blockdiagram_filename)
    shutil.make_archive(self.simulink_library_filename,'zip',self.simulink_library_dirname)
    shutil.move(self.simulink_library_filename+".zip",self.simulink_library_filename)


class Mask:
  def __init__(self,s_function, port_labels_in=None, port_labels_out=None, name=None, port_in=None,port_out=None,dependencies=None):
    self.dependencies = dependencies
    self.s_function = s_function
    self.port_labels_in = port_labels_in
    self.port_labels_out = port_labels_out
    if port_in is None:
      port_in = []
    self.port_in = port_in
    self.name = name
    self.stride_height  = 40
    self.padding_height = 10
    self.unit_height  = 40

    self.margin_left = 100
    self.margin_top = 100

    self.constant_width = 30
    self.constant_height = 30

    self.line_width = 100
    self.sfun_width = 200
    self.sfun_height = self.stride_height*max(len(self.port_labels_in),len(self.port_labels_out))

    self.constant_height_offset = 5
    self.zorder = 7

    mask_commands = []
    for k,e in enumerate(self.port_labels_in):
      mask_commands.append(f"port_label('input',{k+1},'{e}');")
    mask_commands.append(f"disp('{self.name}');")
    for k,e in enumerate(self.port_labels_out):
      mask_commands.append(f"port_label('output',{k+1},'{e}');")

    self.mask_commands = "\n".join(mask_commands)
  def register(self, base):
    num_elem = len(self.port_in)+2
    self.base_id = base
    self.max_id = self.base_id+num_elem

  @property
  def width(self):
    return self.sfun_width+self.constant_width+self.line_width
  @property
  def height(self):
    return self.sfun_height

  def write(self,system,offset_left=0,offset_top=0):
    margin_left = self.margin_left+offset_left
    margin_top = self.margin_top+offset_top

    sfun_margin_left = margin_left+self.constant_width+self.line_width
    sfun_margin_top = margin_top


    s = f"""
    <Block BlockType="S-Function" Name="S-Function{self.base_id}" SID="{self.base_id}">
      <P Name="Ports">[{len(self.port_labels_in)}, {len(self.port_labels_out)}]</P>
      <P Name="Position">[{sfun_margin_left}, {sfun_margin_top}, {sfun_margin_left+self.sfun_width}, {sfun_margin_top+self.sfun_height}]</P>
      <P Name="ZOrder">{self.zorder}</P>
      <P Name="FunctionName">{self.s_function}</P>
      <P Name="SFunctionDeploymentMode">off</P>
      <P Name="EnableBusSupport">off</P>
      <P Name="SFcnIsStateOwnerBlock">off</P>
      <Object PropName="MaskObject" ObjectID="{self.base_id+7}" ClassName="Simulink.Mask">
        <P Name="Display" Class="char">{self.mask_commands}</P>
      </Object>"""
    if self.dependencies:
      s+=f"""<P Name="SFunctionModules">&apos;{" ".join(self.dependencies)}&apos;</P>"""
    s += f"</Block>"
    system.append(etree.fromstring(s))

    for i, (default, label) in enumerate(zip(self.port_in,self.port_labels_in)):
      id = self.base_id+i+2
      if default is None: continue
      zorder = 7
      block = etree.fromstring(f"""
      <Block BlockType="Constant" Name="Constant{id}" SID="{id}">
        <P Name="Position">[{margin_left}, {margin_top+i*self.stride_height+self.constant_height_offset}, {margin_left+self.constant_width}, {margin_top+self.constant_height+i*self.stride_height+self.constant_height_offset}]</P>
        <P Name="ZOrder">{zorder}</P>
        <P Name="Value">{default}</P>
        <P Name="VectorParams1D">off</P>
      </Block>
      """)
      system.append(block)

      block = etree.fromstring(f"""
      <Line>
        <P Name="ZOrder">1</P>
        <P Name="Src">{id}#out:1</P>
        <P Name="Dst">{self.base_id}#in:{i+1}</P>
      </Line>
      """)
      system.append(block)



class MPC(Ocp):
  def __init__(self, **kwargs):
    Ocp.__init__(self, **kwargs)
    self._expr = Model()
    self.basename = os.path.dirname(__file__)
    self._added_functions = []

  @property
  def expr(self):
      return self._expr

  def parameter(self,name,*args,**kwargs):
    p = MX.sym(name,*args)
    self.register_parameter(p,**kwargs)
    self.expr._register('p', {name: p})
    return p

  def add_function(self, fun):
    self._added_functions.append(fun)

  def add_model(self,name,file_name):
    # Read meta information
    with open(file_name) as file:
        model_meta = yaml.load(file, Loader=yaml.FullLoader)

    m = Model(name+".")

    # Define constants
    constants = {}
    definitions = casadi.__dict__
    locals = dict(m._d)
    for k,v in m._d.items():
      if k.startswith(name+"."):
        locals[k[len(name)+1:]] = v
    if "constants" in model_meta:
      if "inline" in model_meta["constants"]:
        for k,v in model_meta["constants"]["inline"].items():
          try:
            constants[k] = eval(v,casadi.__dict__,locals)
          except:
            constants[k] = v
          locals[k] = constants[k]
    m._register("c", constants)

    # Read equations
    if not "equations" in model_meta:
      raise Exception("Model file is missing 'equations' section.")
    equations = model_meta["equations"]
    if ("external" in equations) == ("inline" in equations):
      raise Exception("Model equations must be either 'inline' or 'external'.")

    if "external" in equations:
      external = equations["external"]
      assert external["type"]=="casadi_serialized"
      model_file_name = os.path.expanduser(external["file_name"])
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
    else:
      # Not defined yet, defer until after variables meta-data
      model = None

    # Read variables meta-data
    args = {}
    names = {}
    nd = {}

    # Loop over dae variables, e.g. key=x, long_name=differential_states
    for key, long_name in dae_keys.items():
      # Get corresponding rockit syntax name
      rockit_name = dae_rockit[key]

      # If we have a model already (externally defined), retrieve variable length
      if model:
        var_len = model.numel_in(key) if key in model.name_in() else 0
      else:
        var_len = 0
      if var_len or not model:
        if long_name in model_meta:
          s = m._register(key,model_meta[long_name])
          names[key] = s._names
          locals.update(dict(zip(s._names,s._symbols)))
          v = getattr(m,key)
          if model:
            assert v.numel()==var_len
          else:
            var_len = v.numel()
          register = getattr(self,"register_"+rockit_name)
          for e in v.primitives():
            register(e)
        else:
          if model:
            v = getattr(self,rockit_name)("dummy",var_len)
            #m._register(key, v)
          else:
            v = MX(0, 1)
        args[key] = v
      nd[key] = var_len

    # Define Outputs
    outputs = {}
    definitions = casadi.__dict__
    locals = dict(m._d)
    for k,v in m._d.items():
      if k.startswith(name+"."):
        locals[k[len(name)+1:]] = v
    if "outputs" in model_meta:
      if "inline" in model_meta["outputs"]:
        for k,v in model_meta["outputs"]["inline"].items():
          outputs[k] = eval(v,casadi.__dict__,locals)
    m._register("y", outputs)
    locals.update(outputs)


    if "external" in equations:
      model_res = model(**args)
    
    # Parse inline equations
    if "inline" in equations:
      model_res = {}
      inline = equations["inline"]
      if "ode" in inline:
        ode = inline["ode"]
        rhs_ordered = []
        for local_name in names["x"]:
          if local_name not in ode:
            raise Exception()
          rhs_ordered.append(eval(ode[local_name],casadi.__dict__,locals))
        model_res["ode"] = vvcat(rhs_ordered)
        assert len(names["x"])==len(ode)

      if "alg" in inline:
        alg = inline["alg"]
        alg_ordered = []
        for e in alg:
          alg_ordered.append(eval(e,casadi.__dict__,locals))
        model_res["alg"] = vvcat(alg_ordered)


    if nd["x"]:
      assert "ode" in model_res
      self.set_der(m.x, model_res["ode"])
    if nd["z"]:
      assert "alg" in model_res
      self.add_alg(model_res["alg"])


    self.expr._update({name: m})
    return m

  @staticmethod
  def patch_codegen(name):
    import re
    with open(name,'r') as fin:
      lines = fin.readlines()

    increfs = []

    qp = False

    with open(name,'w') as fout:
      for line in lines:
        # Bugfix https://gitlab.kuleuven.be/meco/projects/sbo_dirac/dirac_mpc/-/issues/11
        if "return fmax(x, y);" not in line and "CASADI_PREFIX" not in line:
          line = re.sub(r"\bfmax\b","casadi_fmax", line)
        # Bugfix https://github.com/casadi/casadi/issues/2835

        m=re.search(r"\bcasadi_f\d+_checkout\b",line)
        if m:
          increfs.append(m.group(0))

        if "ocpfun_checkout" in line:
          fout.write(line)
          line = " ".join([e+"();" for e in increfs])
          
        if "casadi_int A_colind" in line:
          line = line.replace("casadi_int","c_int")

        if "/* Solve the QP */" in line:
          qp = True
        
        if "/* Detecting indefiniteness */" in line:
          qp = False

        if qp and "casadi_f" in line:
          indent = line[:len(line)-len(line.lstrip())]
          line = indent + "if (" + line.strip()[:-1] + ") return 1;\n"

        fout.write(line)


  def export(self,name,src_dir=".",use_codegen=None,context=None,ignore_errors=False,short_output=True):
    build_dir_rel = name+"_build_dir"
    build_dir_abs = os.path.join(os.path.abspath(src_dir),build_dir_rel)

    def escape(e):
      return e.replace('\\','/')

    os.makedirs(build_dir_abs,exist_ok=True)
    # Clean directory (but do not delete it,
    # since this confuses open shells in Linux (e.g. bash, Matlab)
    for filename in os.listdir(build_dir_abs):
      file_path = os.path.join(build_dir_abs, filename)
      try:
        if os.path.isfile(file_path) or os.path.islink(file_path):
          os.unlink(file_path)
        elif os.path.isdir(file_path):
          shutil.rmtree(file_path)
      except:
        pass
    
    print(self.x)
    print(self.z)
    print(self.u)

    print(self.p)

    [_,states] = self.sample(self.x,grid='control')
    [_,algebraics] = self.sample(self.z,grid='control')
    [_,controls] = self.sample(self.u,grid='control-')
    parameters_symbols = self.parameters['']+self.parameters['control']
    parameters = []
    for p in self.parameters['']:
      parameters.append(self.value(p))
    for p in self.parameters['control']:
      parameters.append(self.sample(p,grid='control-')[1])
    casadi_fun_name = 'ocpfun'
    is_coll = False
    if hasattr(self._method, "Xc"):
      is_coll = True

    ocpfun = self.to_function(casadi_fun_name,[states]+(["z"] if is_coll else [MX()])+[controls]+parameters,[states,algebraics,controls])

    casadi_codegen_file_name_base = name+"_codegen.c"
    casadi_codegen_file_name = os.path.join(build_dir_abs,casadi_codegen_file_name_base)


    if self.nz>0:
      sim = self.sys_simulator(intg="collocation",intg_options={"simplify":True,"rootfinder":"fast_newton"})
    else:
      sim = self.sys_simulator(intg="rk",intg_options={"simplify":True})
    sim_args = {}
    args = []
    label_in = []
    args.append(self.x)
    label_in.append("x")
    sim_args["x"] = self.x

    args.append(self.u)
    label_in.append("u")
    sim_args["u"] = self.u
    sim_p = []
    for p,appearing in zip(parameters_symbols,self.is_parameter_appearing_in_sys()):
      if appearing:
        args.append(p)
        sim_p.append(p)
    sim_p = vvcat(sim_p)
    sim_args["p"] = sim_p

    if self.is_sys_time_varying():
      args.append(self.t)
      label_in.append("t")
      sim_args["t0"] = self.t
    dt = MX.sym("dt")
    args.append(dt)
    label_in.append("dt")
    sim_args["dt"] = dt



    if self.nz>0:
      z_initial_guess = MX.sym("z_initial_guess",self.nz)
      args.append(z_initial_guess)
      label_in.append("z_initial_guess")
      sim_args["z_initial_guess"] = z_initial_guess

    sim_out = sim(**sim_args)
    outs = [sim_out["xf"]]
    labels_out = ["xf"]
    if self.nz>0:
      outs.append(sim_out["zf"])
      labels_out.append("zf")
    
    mysim = Function('integrate_'+name, args, outs, label_in, labels_out)

    self.add_function(mysim)

    if use_codegen is None or use_codegen:
      options = {}
      options["with_header"] = True
      cg = CodeGenerator(name+"_codegen", options)
      f = io.StringIO()
      with redirect_stderr(f):
        cg.add(ocpfun)
        cg.generate(build_dir_abs+os.sep)
        data = cg.dump()
      f.seek(0)
      if "cannot be code generated" in f.read() or "#error" in data:
        if use_codegen is None:
          use_codegen = False
        else:
          raise Exception("use_codegen argument was True, but not supported.")
      else:
        use_codegen = True
        self.patch_codegen(casadi_codegen_file_name)
    print("use_codegen", use_codegen)
    casadi_file_name = os.path.join(build_dir_abs,name+".casadi")
    ocpfun.save(casadi_file_name)
    prefix = "impact_"

    def format_float(e):
      return "%0.18f" % e

    def strlist(a):
      elems = []
      for e in a:
        if isinstance(e,str):
          elems.append('"'+e+'"')
        elif isinstance(e,float):
          elems.append(format_float(e))
        else:
          elems.append(str(e))
      return ",".join(elems)


    pool_names = ["x_initial_guess","z_initial_guess","u_initial_guess","p","x_opt","z_opt","u_opt"]

    p_offsets = [0]
    for p in parameters:
      p_offsets.append(p_offsets[-1]+p.numel())

    p_nominal = self._method.opti.value(vvcat(parameters),self._method.opti.initial())
    x_nominal = self._method.opti.value(vec(states),self._method.opti.initial())
    z_nominal = self._method.opti.value(vec(algebraics),self._method.opti.initial())
    u_nominal = self._method.opti.value(vec(controls),self._method.opti.initial())

    if isinstance(p_nominal,float): p_nominal = np.array([p_nominal])
    if isinstance(x_nominal,float): x_nominal = np.array([x_nominal])
    if isinstance(z_nominal,float): z_nominal = np.array([z_nominal])
    if isinstance(u_nominal,float): u_nominal = np.array([u_nominal])

    p_names = [p.name() for p in self.parameters['']+self.parameters['control']]
    x_names = [x.name() for x in self.states]
    z_names = [z.name() for z in self.algebraics]
    u_names = [u.name() for u in self.controls]


    hello_p_normal_name = None
    hello_p_normal = None
    hello_p_normal_nominal = None
    if self.parameters['']:
      hello_p_normal = self.parameters[''][-1]
      hello_p_normal_name = hello_p_normal.name()
      hello_p_normal_nominal = self._method.opti.value(self.value(hello_p_normal),self._method.opti.initial())
      if isinstance(hello_p_normal_nominal,float): hello_p_normal_nominal = np.array([hello_p_normal_nominal])

    i_x_current = None
    count = 0

    p_part_offset = [0]
    p_part_unit = []
    for p_symbol,p_sampled in zip(parameters_symbols,parameters):
      if p_symbol.name()=="x_current":
        i_x_current = count
      count += 1
      p_part_unit.append(p_symbol.numel())
      p_part_offset.append(p_part_offset[-1]+p_sampled.numel())

    if i_x_current is None:
      raise Exception("You must define a parameter named 'x_current'")

    x_current_nominal = self._method.opti.value(parameters[i_x_current],self._method.opti.initial())

    c_file_name_base = name+".c"
    c_file_name = os.path.join(build_dir_abs,c_file_name_base)
    h_file_name_base = name+".h"
    h_file_name = os.path.join(build_dir_abs,h_file_name_base)
    hello_file_name_base = "hello_world_"+name
    hello_file_name = os.path.join(build_dir_abs,hello_file_name_base)
    hello_c_file_name_base = hello_file_name_base+".c"
    hello_c_file_name = hello_file_name + ".c"
    with open(hello_c_file_name,"w") as out:
      out.write(f"""
        #include <{name}.h>
        #include <stdio.h>
        #include <assert.h>

        int main() {{
          {prefix}struct* m = initialize(printf);
          if (!m) {{
            printf("Failed to initialize\\n");
            return 1;
          }}



          int n;

          double t[2] = {{42, 43}};
          
          n = set(m, "x_initial_guess", "scara.joint_vel", IMPACT_EVERYWHERE, t, IMPACT_HREP);
          assert(n==2);
          print_problem(m);
          t[0] = 3;t[1] = 7;
          n = set(m, "x_initial_guess", "scara.joint_vel", 2, t, IMPACT_HREP);
          assert(n==2);
          print_problem(m);

          double t4[4] = {{0.1, 0.2, 0.3, 0.4}};

          n = set(m, "x_initial_guess", IMPACT_ALL, 3, t4, IMPACT_HREP);
          assert(n==4);
          print_problem(m);

          double t44[44];
          for (int i=0;i<44;++i) {{
            t44[i] = i;
          }}
          n = set(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, t44, IMPACT_FULL);
          assert(n==44);
          print_problem(m);

          n = set(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, t44, IMPACT_FULL | IMPACT_ROW_MAJOR);
          assert(n==44);
          print_problem(m);

          n = set(m, "x_initial_guess", "scara.joint_vel", IMPACT_EVERYWHERE, t44, IMPACT_FULL | IMPACT_ROW_MAJOR);
          assert(n==22);
          print_problem(m);

          n = get(m, "x_initial_guess", "scara.joint_vel", 2, t, IMPACT_HREP);
          assert(n==2);
          assert(t[0]==2.0);
          assert(t[1]==13.0);
          n = get(m, "x_initial_guess", "scara.joint_vel", 2, t, IMPACT_FULL);
          assert(n==2);
          assert(t[0]==2.0);
          assert(t[1]==13.0);

          int n_row;
          int n_col;
          {prefix}get_size(m, "u_opt", IMPACT_ALL, &n_row, &n_col);
          printf("dims %d %d\\n",n_row,n_col);

          casadi_int sz_arg, sz_res, sz_iw, sz_w;



          work(m, &sz_arg, &sz_res, &sz_iw, &sz_w);
          printf("work %lld %lld %lld %lld\\n",sz_arg,sz_res,sz_iw,sz_w);


          solve(m);

          destroy(m);
        }}
      """
      )

    with open(hello_c_file_name,"w") as out:
      out.write(f"""
        #include <{name}.h>
        #include <stdio.h>
        #include <stdlib.h>
        #include <assert.h>

        int main() {{
          int i, j, n_row, n_col;
          double *u_scratch, *x_scratch;
          {prefix}struct* m = impact_initialize(printf);
          if (!m) {{
            printf("Failed to initialize\\n");
            return 1;
          }}
      """)
      if hello_p_normal_name:
        out.write(f"""
          /* Example: how to set a parameter */
          double val_{hello_p_normal_name}[{len(hello_p_normal_nominal)}] = {{{strlist(hello_p_normal_nominal)}}};
          impact_set(m, "p", "{hello_p_normal_name}", IMPACT_EVERYWHERE, val_{hello_p_normal_name}, IMPACT_FULL);

        """)
      out.write(f"""
          double x0[{x_current_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_current_nominal)} }};

          impact_set(m, "x_current", IMPACT_ALL, IMPACT_EVERYWHERE, x0, IMPACT_FULL);

          double num = {"%0.16f" % x_current_nominal[0]};

          impact_set(m, "x_current", "{self.states[0].name()}", IMPACT_EVERYWHERE, &num, IMPACT_FULL);

          impact_print_problem(m);

          impact_solve(m);


          /* Allocate scratch space for state and control trajectories */
          impact_get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
          printf("u_opt dims: %d - %d\\n", n_row, n_col);
          /* u_scratch = malloc(sizeof(double)*n_row*n_col); */
          u_scratch = malloc(sizeof(double)*n_row);

          impact_get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
          printf("x_opt dims: %d - %d\\n", n_row, n_col);
          x_scratch = malloc(sizeof(double)*n_row*n_col);

          printf("Single OCP\\n");

          impact_get(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, x_scratch, IMPACT_FULL);
          for (i=0;i<n_col;++i) {{
            for (j=0;j<n_row;++j) {{
              printf("%0.3e ", x_scratch[i*n_row+j]);
            }}
            printf("\\n");
          }}

          free(x_scratch);

          x_scratch = malloc(sizeof(double)*n_row);

          printf("Simulated MPC\\n");
  
          for (i=0;i<100;++i) {{
            impact_solve(m);
            impact_get(m, "u_opt", IMPACT_ALL, 0, u_scratch, IMPACT_FULL);
            impact_get(m, "x_opt", IMPACT_ALL, 1, x_scratch, IMPACT_FULL);
            impact_set(m, "x_current", IMPACT_ALL, 0, x_scratch, IMPACT_FULL);

            for (j=0;j<n_row;++j) {{
              printf("%0.3e ", x_scratch[j]);
            }}
            printf("\\n");

          }}

          impact_destroy(m);
        }}
      """
      )
    flags = OrderedDict([("ALL",0),("EVERYWHERE",-1),("FULL",0),("HREP",1),("VREP",2),("COLUMN_MAJOR",0),("ROW_MAJOR",4)])

    with open(h_file_name,"w") as out:
      for flag_name, flag_value in flags.items():
        out.write(f"#define IMPACT_{flag_name} {flag_value}\n")

      out.write(f"""

struct {prefix}_struct;
typedef struct {prefix}_struct {prefix}struct;

typedef int (*formatter)(const char * s);

/**
  \\brief Initialize and instance of an impact library

  \\note Any call to \\c initialize must be followed up with a call to \\c destroy within the same process.

  \\param[in]  fp Function pointer to a printf-like function
              Common values: 0 (null pointer: no printing), printf (C's standard printf method defined in stdio)


  \\return Impact instance pointer, or a null pointer upon failure

*/
{prefix}struct* {prefix}initialize(formatter fp);
/**
  \\brief Destroy impact instance

  \\note May be needed to avoid segementation faults on exit
  \\note It is safe to pass a null pointer as argument

  \\parameter[in] Impact instance pointer
*/
void {prefix}destroy({prefix}struct* m);

/**
  \\brief Compute an MPC action

  \details
  Parameters (\\b p) and initial guesses (\\b x_initial_guess, \\b u_initial_guess, \\b z_initial_guess)
  can be supplied through \\c get.
  If not supplied, defaults will be taken from the user definition.
  Outputs of the optimization are written into internal pools \\b x_opt, \\b z_opt, and \\b u_opt,
  and can be queried using \\c get. 

  \\parameter[in] Impact instance pointer

  \\return 0 indicates success

*/
int {prefix}solve({prefix}struct* m);

/**
  \\brief Print numerical data present in pools

  \\parameter[in] Impact instance pointer

  \\return 0 indicates success

*/
int {prefix}print_problem({prefix}struct* m);

/** \\brief Getting and setting numerical data

  \\parameter[in] m            Impact instance pointer
  \\parameter[in] pool_name    Any of: { strlist(pool_names) }
  \\parameter[in] id           String identifier for a specific variable name, or IMPACT_ALL
  \\parameter[in] stage        Specific time instance (0-based), or IMPACT_EVERYWHERE
  \\parameter[in] flags    Composable (using bitwise or '|' ) flags that modify behaviour
                            IMPACT_FULL: default
                            IMPACT_COLUMN_MAJOR (fortran ordering - default)
                            IMPACT_ROW_MINOR (C ordering)
                            IMPACT_HREP (repeat horizontally )

  \\return Negative number upon failure

 */
 /** @{{ */
/**
  \\brief Get numerical data
  \\parameter[in] dst          Pointer to a chunk of writable memory.
                               There must be space of at least \\b n_row* \\b n_col which can be retrieved using \\c get_size
*/
int {prefix}get({prefix}struct* m, const char* pool_name, const char* id, int stage, double* dst, int flags);
/**
  \\brief Set numerical data
  \\parameter[in] src          Pointer to a chunk of readable memory.
                               There will be \\b n_row* \\b n_col elements read which can be retrieved using \\c get_size
*/
int {prefix}set({prefix}struct* m, const char* pool_name, const char* id, int stage, const double* src, int flags);
/**
  \\brief Get shape of data
*/
int {prefix}get_size({prefix}struct* m, const char* pool_name, const char* id, int stage, int flags, int* n_row, int* n_col);
/* @}} */

/**
  \\brief Get number of (scalarized) differential states
*/
int {prefix}get_nx();
/**
  \\brief Get number of (scalarized) algebric states
*/
int {prefix}get_nz();
/**
  \\brief Get number of (scalarized) inputs
*/
int {prefix}get_nu();

int {prefix}get_id_count({prefix}struct* m, const char* pool_name);
int {prefix}get_id_from_index({prefix}struct* m, const char* pool_name, int index, const char** id);

#define casadi_real double
#define casadi_int long long int

int {prefix}allocate({prefix}struct* m);
void {prefix}set_work({prefix}struct* m, const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w);
void {prefix}work({prefix}struct* m, casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);

int {prefix}flag_size({prefix}struct* m);
const char* {prefix}flag_name({prefix}struct* m, int index);
int {prefix}flag_value({prefix}struct* m, int index);

      """
      )


    x_part_offset = [0]
    x_part_unit = []
    for x in self.states:
      x_part_unit.append(x.numel())
      x_part_offset.append(x_part_offset[-1]+x.numel())

    z_part_offset = [0]
    z_part_unit = []
    for z in self.algebraics:
      z_part_unit.append(z.numel())
      z_part_offset.append(z_part_offset[-1]+z.numel())

    u_part_offset = [0]
    u_part_unit = []
    for u in self.controls:
      u_part_unit.append(u.numel())
      u_part_offset.append(u_part_offset[-1]+u.numel())

    p_trajectory_length = [1 for p in self.parameters['']]+[self._method.N for p in self.parameters['control']]
    x_trajectory_length = [self._method.N+1 for x in self.states]
    z_trajectory_length = [self._method.N+1 for x in self.algebraics]
    u_trajectory_length = [self._method.N for x in self.controls]
    x_current_trajectory_length = [1 for x in self.states]

    p_part_stride = p_part_unit
    x_part_stride = [self.nx for x in self.states]
    z_part_stride = [self.nz for x in self.algebraics]
    u_part_stride = [self.nu for u in self.controls]

    def array_size(n):
      if n==0: return 1
      return n

    def initialize_array(name,n,init):
      s = name+f"[{array_size(n)}]"
      if n==0:
        return s
      else:
        return s + " = {" + str(init) + "}"

    def casadi_call(funname,*args):
      method = f"casadi_c_{funname}_id"
      if use_codegen:
        if funname=="eval":
          method = f"{casadi_fun_name}"
        else:
          method = f"{casadi_fun_name}_{funname}"
      if not use_codegen:
        args = ("m->id",) + args
      return method + "(" + ",".join(args) + ")"

    with open(c_file_name,"w") as out:
      out.write(f"""
          #include <stdlib.h>
          #include <string.h>          
          {"#include " + '"' + name + "_codegen.h" + '"' if use_codegen else "#include <casadi/casadi_c.h>"}
          #include "{name}.h"

          typedef struct {prefix}_pool {{
            casadi_int n;
            casadi_int size;            
            const char** names; /* length n */
            const casadi_int* trajectory_length;  /*  length n */
            const casadi_int* part_offset;  /*  length n */
            const casadi_int* part_unit;  /*  length n a non-full may span multiple parts */
            const casadi_int* part_stride;  /*  length n a non-full may span multiple parts */
            casadi_real* data;
            casadi_int stride;
          }} {prefix}pool;

          typedef void (*fatal_fp)({prefix}struct* m, const char * loc, const char * fmt, ...);
          typedef void (*info_fp)({prefix}struct* m, const char * fmt, ...);

          struct {prefix}_struct {{
            int id;
            int pop; /*  need to pop when destroyed? */
            casadi_int n_in;
            casadi_int n_out;

            casadi_int sz_arg;
            casadi_int sz_res;
            casadi_int sz_iw;
            casadi_int sz_w;

            const casadi_real** arg;
            casadi_real** res;
            casadi_int* iw;
            casadi_real* w;

            const casadi_real** arg_casadi;
            casadi_real** res_casadi;
            casadi_int* iw_casadi;
            casadi_real* w_casadi;

            {prefix}pool* x_current;

            {prefix}pool* x_initial_guess;
            {prefix}pool* z_initial_guess;
            {prefix}pool* u_initial_guess;
            {prefix}pool* p;


            {prefix}pool* x_opt;
            {prefix}pool* z_opt;
            {prefix}pool* u_opt;

            int mem;

            formatter fp;
            fatal_fp fatal;
            info_fp info;
          }};

          /* For printing */
          #include <stdio.h>
          #include <stdarg.h>
""")
      # Write all data arrays
      prim = ["x","z","u","p"]
      def arrays():
        yield ("p_offsets","casadi_int")
        for p in prim: yield (f"{p}_part_offset","casadi_int")
        for p in prim: yield (f"{p}_part_unit","casadi_int")
        for p in prim: yield (f"{p}_part_stride","casadi_int")
        for p in prim: yield (f"{p}_names","char*")
        for p in prim: yield (f"{p}_trajectory_length","casadi_int")
        yield ("x_current_trajectory_length","casadi_int")
        for p in prim: yield (f"{p}_nominal","casadi_real")
        yield ("x_current_nominal","casadi_real")

      for array_name, data_type in arrays():
        data = eval(array_name)
        out.write(f"          static const {data_type} "+ initialize_array(prefix+array_name,len(data),strlist(data))+";\n")

      out.write(f"""
          static const char* {prefix}pool_names[{array_size(len(pool_names))}] = {{ {strlist(pool_names)} }};

          static const char* {prefix}flag_names[{len(flags)}] = {{ {",".join('"%s"' % e for e in flags.keys())} }};
          static int {prefix}flag_values[{len(flags)}] = {{ {",".join("IMPACT_" + e for e in flags.keys())} }};

          int {prefix}flag_size({prefix}struct* m) {{
            return {len(flags)};
          }}
          const char* {prefix}flag_name({prefix}struct* m, int index) {{
            if (index<0 || index>={len(flags)}) return 0;
            return {prefix}flag_names[index];
          }}
          int {prefix}flag_value({prefix}struct* m, int index) {{
            if (index<0 || index>={len(flags)}) return 0;
            return {prefix}flag_values[index];
          }}



          void {prefix}fatal({prefix}struct* m, const char* loc, const char* fmt, ...) {{
            va_list args;
            char buffer[1024];
            if (m->fp) {{
              va_start(args, fmt);
              if (loc) {{
                sprintf(buffer, "Error in %s: ", loc);
                m->fp(buffer);
              }}
              vsprintf(buffer, fmt, args);
              m->fp(buffer);
              va_end(args);
            }}
          }}
          void {prefix}info({prefix}struct* m, const char* fmt, ...) {{
            va_list args;
            char buffer[1024];
            if (m->fp) {{
              va_start(args, fmt);
              vsprintf(buffer, fmt, args);
              m->fp(buffer);
              va_end(args);
            }}
          }}


          {prefix}struct* {prefix}initialize(formatter fp) {{
            int flag;
            int i;
            {prefix}struct* m;
            m = malloc(sizeof({prefix}struct));
            m->fp = fp;
            m->fatal = {prefix}fatal;
            m->info = {prefix}info;
            m->id = -1;""")
      if not use_codegen:
        out.write(f"""
            if (casadi_c_n_loaded()) {{ 
              m->id = casadi_c_id("{casadi_fun_name}");
              m->pop = 0;
            }}
            if (m->id<0) {{
              flag = casadi_c_push_file("{escape(casadi_file_name)}");
              m->pop = 1;
              if (flag) {{
                m->fatal(m, "initialize", "Could not load file '{escape(casadi_file_name)}'.\\n");
                return 0;
              }}
              m->id = casadi_c_id("{casadi_fun_name}");
              if (m->id<0) {{
                m->fatal(m, "initialize", "Could not locate function with name '{casadi_fun_name}'.\\n");
                {prefix}destroy(m);
                return 0;
              }}
            }}""")
      out.write(f"""
            /* Allocate memory (thread-safe) */
            {casadi_call("incref")};
            /* Checkout thread-local memory (not thread-safe) */
            m->mem = {casadi_call("checkout")};

            {prefix}work(m, &m->sz_arg, &m->sz_res, &m->sz_iw, &m->sz_w);
            {prefix}set_work(m,
              malloc(sizeof(const casadi_real*)*m->sz_arg),
              malloc(sizeof(casadi_real*)*m->sz_res),
              malloc(sizeof(casadi_int)*m->sz_iw),
              malloc(sizeof(casadi_real)*m->sz_w));

            m->arg_casadi = m->arg;
            m->res_casadi = m->res;
            m->iw_casadi = m->iw;
            m->w_casadi = m->w;

            m->p = malloc(sizeof({prefix}pool));
            m->p->names = {prefix}p_names;
            m->p->size = {p_nominal.size};
            m->p->data = malloc(sizeof(casadi_real)*{p_nominal.size});
            m->p->n = {len(parameters)};
            m->p->trajectory_length = {prefix}p_trajectory_length;
            m->p->stride = -1;
            m->p->part_offset = {prefix}p_part_offset;
            m->p->part_unit = {prefix}p_part_unit;
            m->p->part_stride = {prefix}p_part_stride;

            m->x_current = malloc(sizeof({prefix}pool));
            m->x_current->names = {prefix}x_names;
            m->x_current->size = {self.nx};
            m->x_current->data = m->p->data+m->p->part_offset[{i_x_current}];
            m->x_current->n = {len(self.states)};
            m->x_current->trajectory_length = {prefix}x_current_trajectory_length;
            m->x_current->stride = {self.nx};
            m->x_current->part_offset = {prefix}x_part_offset;
            m->x_current->part_unit = {prefix}x_part_unit;
            m->x_current->part_stride = {prefix}x_part_stride;

            m->x_initial_guess = malloc(sizeof({prefix}pool));
            m->x_initial_guess->names = {prefix}x_names;
            m->x_initial_guess->size = {x_nominal.size};
            m->x_initial_guess->data = malloc(sizeof(casadi_real)*{x_nominal.size});
            m->x_initial_guess->n = {len(self.states)};
            m->x_initial_guess->trajectory_length = {prefix}x_trajectory_length;
            m->x_initial_guess->stride = {self.nx};
            m->x_initial_guess->part_offset = {prefix}x_part_offset;
            m->x_initial_guess->part_unit = {prefix}x_part_unit;
            m->x_initial_guess->part_stride = {prefix}x_part_stride;

            m->z_initial_guess = malloc(sizeof({prefix}pool));
            m->z_initial_guess->names = {prefix}z_names;
            m->z_initial_guess->size = {z_nominal.size};
            m->z_initial_guess->data = malloc(sizeof(casadi_real)*{z_nominal.size});
            m->z_initial_guess->n = {len(self.algebraics)};
            m->z_initial_guess->trajectory_length = {prefix}z_trajectory_length;
            m->z_initial_guess->stride = {self.nz};
            m->z_initial_guess->part_offset = {prefix}z_part_offset;
            m->z_initial_guess->part_unit = {prefix}z_part_unit;
            m->z_initial_guess->part_stride = {prefix}z_part_stride;

            m->u_initial_guess = malloc(sizeof({prefix}pool));
            m->u_initial_guess->names = {prefix}u_names;
            m->u_initial_guess->size = {u_nominal.size};
            m->u_initial_guess->data = malloc(sizeof(casadi_real)*{u_nominal.size});
            m->u_initial_guess->n = {len(self.controls)};
            m->u_initial_guess->trajectory_length = {prefix}u_trajectory_length;
            m->u_initial_guess->stride = {self.nu};
            m->u_initial_guess->part_offset = {prefix}u_part_offset;
            m->u_initial_guess->part_unit = {prefix}u_part_unit;
            m->u_initial_guess->part_stride = {prefix}u_part_stride;

            m->u_opt = malloc(sizeof({prefix}pool));
            m->u_opt->names = {prefix}u_names;
            m->u_opt->size = {u_nominal.size};
            m->u_opt->data = malloc(sizeof(casadi_real)*{u_nominal.size});
            m->u_opt->n = {len(self.controls)};
            m->u_opt->trajectory_length = {prefix}u_trajectory_length;
            m->u_opt->stride = {self.nu};
            m->u_opt->part_offset = {prefix}u_part_offset;
            m->u_opt->part_unit = {prefix}u_part_unit;
            m->u_opt->part_stride = {prefix}u_part_stride;

            m->z_opt = malloc(sizeof({prefix}pool));
            m->z_opt->names = {prefix}z_names;
            m->z_opt->size = {z_nominal.size};
            m->z_opt->data = malloc(sizeof(casadi_real)*{z_nominal.size});
            m->z_opt->n = {len(self.algebraics)};
            m->z_opt->trajectory_length = {prefix}z_trajectory_length;
            m->z_opt->stride = {self.nz};
            m->z_opt->part_offset = {prefix}z_part_offset;
            m->z_opt->part_unit = {prefix}z_part_unit;
            m->z_opt->part_stride = {prefix}z_part_stride;

            m->x_opt = malloc(sizeof({prefix}pool));
            m->x_opt->names = {prefix}x_names;
            m->x_opt->size = {x_nominal.size};
            m->x_opt->data = malloc(sizeof(casadi_real)*{x_nominal.size});
            m->x_opt->n = {len(self.states)};
            m->x_opt->trajectory_length = {prefix}x_trajectory_length;
            m->x_opt->stride = {self.nx};
            m->x_opt->part_offset = {prefix}x_part_offset;
            m->x_opt->part_unit = {prefix}x_part_unit;
            m->x_opt->part_stride = {prefix}x_part_stride;

            memcpy(m->p->data, {prefix}p_nominal, {p_nominal.size}*sizeof(casadi_real));
            memcpy(m->x_initial_guess->data, {prefix}x_nominal, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->z_initial_guess->data, {prefix}z_nominal, {z_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_initial_guess->data, {prefix}u_nominal, {u_nominal.size}*sizeof(casadi_real));
            memcpy(m->x_opt->data, {prefix}x_nominal, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->z_opt->data, {prefix}z_nominal, {z_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_opt->data, {prefix}u_nominal, {u_nominal.size}*sizeof(casadi_real));
            return m;
          }}

          void {prefix}destroy({prefix}struct* m) {{
            if (m) {{
              /* Free memory (thread-safe) */
              {casadi_call("decref")};
              /* Release thread-local (not thread-safe) */
              {casadi_call("release","m->mem")};
              {"" if use_codegen else "if (m->pop) casadi_c_pop();"}
              free(m);
            }}
          }}

          void {prefix}set_work({prefix}struct* m, const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w) {{
            m->arg = arg;
            m->res = res;
            m->iw = iw;
            m->w = w;
          }}

          void {prefix}work({prefix}struct* m, casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w) {{
            {casadi_call("work","sz_arg", "sz_res", "sz_iw", "sz_w")};
            /* We might want to be adding other working memory here */
          }}

          const {prefix}pool* {prefix}get_pool_by_name({prefix}struct* m, const char* name) {{
            if (!strcmp(name,"p")) {{
              return m->p;
            }} else if (!strcmp(name,"x_current")) {{
              return m->x_current;
            }} else if (!strcmp(name,"x_initial_guess")) {{
              return m->x_initial_guess;
            }} else if (!strcmp(name,"z_initial_guess")) {{
              return m->z_initial_guess;
            }} else if (!strcmp(name,"u_initial_guess")) {{
              return  m->u_initial_guess;
            }} else if (!strcmp(name,"x_opt")) {{
              return m->x_opt;
            }} else if (!strcmp(name,"z_opt")) {{
              return m->z_opt;
            }} else if (!strcmp(name,"u_opt")) {{
              return  m->u_opt;
            }} else {{
              m->fatal(m, "get_pool_by_name", "Pool with name '%s' not recognized. \
                                       Use one of: 'p','x_initial_guess','z_initial_guess','u_initial_guess','x_opt','z_opt','u_opt'. \\n", name);
              return 0;
            }}
          }}


          int {prefix}get_id_count({prefix}struct* m, const char* pool) {{
            const {prefix}pool* p;
            if (!pool) {{
              m->fatal(m, "get_id_count (ret code -1)", "You may not pass a null pointer as pool. \\n");
              return -1;
            }}
            p = {prefix}get_pool_by_name(m, pool);
            if (!p) {{
              m->fatal(m, "get_id_count (ret code -2)", "Failed to find a pool named '%s'. \\n", pool);
              return -2;
            }}
            return p->n;
          }}

          int {prefix}get_id_from_index({prefix}struct* m, const char* pool, int index, const char** id) {{
            int i;
            const {prefix}pool* p;
            if (!pool) {{
              m->fatal(m, "get_id_from_index (ret code -1)", "You may not pass a null pointer as pool. \\n");
              return -1;
            }}
            p = {prefix}get_pool_by_name(m, pool);
            if (!p) {{
              m->fatal(m, "get_id_from_index (ret code -2)", "Failed to find a pool named '%s'. \\n", pool);
              return -2;
            }}
            if (index<0 || index>= p->n) {{
              m->fatal(m, "get_id_from_index (ret code -3)", "Index %d is out of bounds for pool %s: need [0,%d[. \\n", index, pool, p->n);
              return -3;
            }}
            *id = p->names[index];

            return 0;
          }}


          int {prefix}get_size({prefix}struct* m, const char* pool, const char* id, int stage, int flags, int* n_row, int* n_col) {{
            int index, i;
            const {prefix}pool* p;
            if (!pool) {{
              m->fatal(m, "get_size (ret code -1)", "You may not pass a null pointer as pool. \\n");
              return -1;
            }}
            p = {prefix}get_pool_by_name(m, pool);
            if (!p) {{
              m->fatal(m, "get_size (ret code -2)", "Failed to find a pool named '%s'. \\n", pool);
              return -2;
            }}

            /* Determine index */
            index = -1;
            if (id!=IMPACT_ALL) {{
              for (i=0;i<p->n;++i) {{
                if (!strcmp(id, p->names[i])) break;
              }}
              if (i==p->n) {{
                m->fatal(m, "get_id_from_index (ret code -4)", "Id '%s' not found for pool '%s'. Use one of these options: \\n", id, pool);
                for (i=0;i<p->n;++i) {{
                  m->fatal(m, 0, " - %s\\n", p->names[i]);
                }}
                return -4;
              }}
              index = i;
            }}

            if (index==-1) {{
              *n_row = p->stride;
              *n_col = p->trajectory_length[0];
            }} else {{
              *n_row = p->part_unit[index];
              *n_col = p->trajectory_length[index];
            }}

            if (stage!=IMPACT_EVERYWHERE) *n_col = 1;

            return 0;
          }}

          int {prefix}get_nx() {{ return {self.nx}; }}
          int {prefix}get_nz() {{ return {self.nz}; }}
          int {prefix}get_nu() {{ return {self.nu}; }}

          int {prefix}set_get({prefix}struct* m, const char* pool, const char* id, int stage, double* data, int data_flags, char mode) {{
            int i, j, k, index, i_start, i_stop, k_start, k_stop, offset, row, col, stride, data_i;
            const {prefix}pool* p;
            if (!pool) {{
              m->fatal(m, "set_get (ret code -1)", "You may not pass a null pointer as pool. \\n");
              return -1;
            }}
            if (!data) {{
              m->fatal(m, "set_get (ret code -2)", "You may not pass a null pointer for src/dst. \\n");
              return -2;
            }}
            p = {prefix}get_pool_by_name(m, pool);
            if (!p) {{
              m->fatal(m, "set_get (ret code -3)", "Failed to find a pool named '%s'. \\n", pool);
              return -3;
            }}

            /* Determine index */
            index = -1;
            if (id!=IMPACT_ALL) {{
              for (i=0;i<p->n;++i) {{
                if (!strcmp(id, p->names[i])) break;
              }}
              if (i==p->n) {{
                m->fatal(m, "set_get (ret code -4)", "Id '%s' not found for pool '%s'. Use one of these options: \\n", id, pool);
                for (i=0;i<p->n;++i) {{
                  m->fatal(m, 0, " - %s\\n", p->names[i]);
                }}
                return -4;
              }}
              index = i;
            }}

            i_start = index==-1? 0 : index;
            i_stop  = index==-1? p->n : index+1;
            k_start = stage==IMPACT_EVERYWHERE? 0 : stage;
            offset = 0;
            data_i = -1;
            for (i=i_start;i<i_stop;++i) {{
              row = id==IMPACT_ALL ? p->part_offset[i] : 0;
              if (data_flags & IMPACT_ROW_MAJOR) {{
                stride = p->trajectory_length[i];
              }} else {{
                stride = id==IMPACT_ALL ? p->part_stride[i] : p->part_unit[i];
              }}
              for (j=0;j<p->part_unit[i];++j) {{
                k_stop  = stage==IMPACT_EVERYWHERE? p->trajectory_length[i] : stage+1;
                for (k=k_start;k<k_stop;++k) {{
                  col = (data_flags & IMPACT_HREP || stage!=IMPACT_EVERYWHERE) ? 0 : k;
                  data_i = data_flags & IMPACT_ROW_MAJOR ? stride*row + col : row + stride*col;
                  if (mode) {{
                    data[data_i] = p->data[j+ p->part_offset[i] + p->part_stride[i]*k];
                  }} else {{
                    /* skip nan */
                    if (data[data_i]==data[data_i]) p->data[j+ p->part_offset[i] + p->part_stride[i]*k] = data[data_i];
                  }}
                }}
                row++;
              }}
            }}
            return data_i+1;
          }}

          int {prefix}set({prefix}struct* m, const char* pool, const char* id, int stage, const double* src, int src_flags) {{
            return {prefix}set_get(m, pool, id, stage, (double*) src, src_flags, 0);
          }}

          int {prefix}get({prefix}struct* m, const char* pool, const char* id, int stage, double* dst, int dst_flags) {{
            return {prefix}set_get(m, pool, id, stage, dst, dst_flags, 1);
          }}

          int {prefix}get_pool(const {prefix}pool* p, casadi_real* value) {{
            if (!value) return 1;
            memcpy(value, p->data, p->size*sizeof(casadi_real));
            return 0;
          }}

          int {prefix}print_problem({prefix}struct* m) {{
            int i,j,k,l,max_len;
            char formatbuffer[10];
            const {prefix}pool* p;
            if (!m->fp) return 1;
            max_len=0;
            for (l=0;l<5;++l) {{
              p = {prefix}get_pool_by_name(m, {prefix}pool_names[l]);
              for (i=0;i<p->n;++i) {{
                max_len = strlen(p->names[i])>max_len? strlen(p->names[i]) : max_len;
              }}
            }}

            if (m->fp) {{
              for (l=0;l<5;++l) {{
                const {prefix}pool* p = {prefix}get_pool_by_name(m, {prefix}pool_names[l]);
                m->info(m, "=== %s ===\\n", {prefix}pool_names[l]);
                sprintf(formatbuffer, "%%-%ds", max_len);
                for (i=0;i<p->n;++i) {{
                  m->info(m, formatbuffer, p->names[i]);
                  m->info(m, ":");
                  for (j=0;j<p->part_unit[i];++j) {{
                    if (j>0) {{
                        m->info(m, formatbuffer, "");
                        m->info(m, " ");
                    }}
                    for (k=0;k<p->trajectory_length[i];++k) {{
                      m->info(m, " %.4e", p->data[j+ p->part_offset[i] + p->part_stride[i]*k]);
                    }}
                    m->info(m, "\\n");
                  }}
                }}
              }}
            }}
            return 0;
          }}

          int {prefix}solve({prefix}struct* m) {{
            int i;
            m->arg_casadi[0] = m->x_initial_guess->data;
            m->arg_casadi[1] = m->z_initial_guess->data;
            m->arg_casadi[2] = m->u_initial_guess->data;
            for (i=0;i<{len(parameters)};i++) {{
              m->arg_casadi[3+i] = m->p->data + {prefix}p_offsets[i];
            }}
            m->res_casadi[0] = m->x_opt->data;
            m->res_casadi[1] = m->z_opt->data;
            m->res_casadi[2] = m->u_opt->data;
            return {casadi_call("eval","m->arg_casadi","m->res_casadi","m->iw_casadi","m->w_casadi","m->mem")};
          }}

          int {prefix}get_p_by_id({prefix}struct* m, const char* id, const casadi_real* value) {{
            return 0;
          }}

          void {prefix}get_u({prefix}struct* m, double* value) {{

          }}

        """)

    lib_name = name
    lib_file_name = os.path.join(build_dir_abs,"lib" + lib_name + ".so")


    s_function_name = name+"_s_function_level2"

    s_function_file_name_base = s_function_name+".c"
    s_function_file_name = os.path.join(build_dir_abs,s_function_file_name_base)
      
    with open(s_function_file_name,"w") as out:
      out.write(f"""
        #define S_FUNCTION_NAME {s_function_name}
        #define S_FUNCTION_LEVEL 2

        #include <{name}.h>

        #include "simstruc.h"

        static {prefix}struct* m;
        struct {{
          int sz_arg;
          int n_p;
        }} config;

        void cleanup() {{
          if (m) {{
            {prefix}destroy(m);
          }}
        }}

        static void mdlInitializeSizes(SimStruct *S) {{
#ifdef MATLAB_MEX_FILE
            mexPrintf("mdlInitializeSizes\\n");
#endif
            int i;
            int n_row, n_col;
            const char* id;
            casadi_int sz_arg, sz_res, sz_iw, sz_w;
            m = 0;

            /* Simulink does not provide a cleanup-hook when parameters are changed */
            cleanup();

#ifdef MATLAB_MEX_FILE
            m = {prefix}initialize(mexPrintf);
#else
            m = {prefix}initialize(0);
#endif
            if (!m) {{
              cleanup();
#ifdef MATLAB_MEX_FILE
              mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                                "Failed to initialize.");
#else
              return;
#endif           
            }}
            {prefix}print_problem(m);

            /* Read work vector requirements */
            {prefix}work(m, &sz_arg, &sz_res, &sz_iw, &sz_w);

            /* Set up CasADi function work vector sizes */
            ssSetNumRWork(S, sz_w);
            ssSetNumIWork(S, sz_iw*sizeof(casadi_int)/sizeof(int_T));
            ssSetNumPWork(S, sz_arg+sz_res);

            int n_p = {prefix}get_id_count(m, "p");
            if (!ssSetNumInputPorts(S, n_p+{3 if self.nz>0 else 2})) return;
            if (n_p<0) {{
              cleanup();
#ifdef MATLAB_MEX_FILE
              mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                                "Failure.");
#else
              return;
#endif            
            }}
            for (i=0;i<n_p;++i) {{
              {prefix}get_id_from_index(m, "p", i, &id);
              {prefix}get_size(m, "p", id, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
              ssSetInputPortDirectFeedThrough(S, i, 1);
              ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
              ssSetInputPortRequiredContiguous(S, i, 1);
            }}

            {prefix}get_size(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            ssSetInputPortRequiredContiguous(S, i, 1);
            i++;""")

      if self.nz>0:
        out.write(f"""
            {prefix}get_size(m, "z_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            ssSetInputPortRequiredContiguous(S, i, 1);
            i++;""")

      out.write(f"""
            {prefix}get_size(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            ssSetInputPortRequiredContiguous(S, i, 1);
            i++;

            if (!ssSetNumOutputPorts(S, {(3 if self.nz>0 else 2) + (1 if ignore_errors else 0)})) return;

            i = 0;
            {prefix}get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, {'1' if short_output else 'n_col'});
            i++;""")

      if self.nz>0:
        out.write(f"""
            {prefix}get_size(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, n_col);
            i++;""")

      out.write(f"""
            {prefix}get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, {'1' if short_output else 'n_col'});
            i++;""")

      if ignore_errors:
        out.write(f"""
            ssSetOutputPortMatrixDimensions(S, i, 1, 1);
            i++;""")

      out.write(f"""
            ssSetNumSampleTimes(S, 1);
            
            ssSetNumNonsampledZCs(S, 0);

            /* specify the sim state compliance to be same as a built-in block */
            ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

            /* Make sure mdlTerminate is called on error */
            ssSetOptions(S,
                        SS_OPTION_EXCEPTION_FREE_CODE |
                        SS_OPTION_USE_TLC_WITH_ACCELERATOR);
        }}

        /* Function: mdlInitializeSampleTimes =========================================
        * Abstract:
        *    Specifiy that we inherit our sample time from the driving block.
        */
        static void mdlInitializeSampleTimes(SimStruct *S)
        {{
#ifdef MATLAB_MEX_FILE
            mexPrintf("mdlInitializeSampleTimes\\n");
#endif
            ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
            ssSetOffsetTime(S, 0, 0.0);
            ssSetModelReferenceSampleTimeDefaultInheritance(S); 
        }}

        static void mdlOutputs(SimStruct *S, int_T tid)
        {{
            if (!m) {{
#ifdef MATLAB_MEX_FILE
              mexPrintf("mdlOutputs (nullptr)\\n");
#endif
              return;
            }}
            void** p;
            const real_T** arg;
            double* w;
            casadi_int* iw;
            int_T i;
            const char* id;
            
#ifdef MATLAB_MEX_FILE
            mexPrintf("mdlOutputs %p\\n", m);
#endif
            /* Set up CasADi function work vectors */
            p = ssGetPWork(S);
            arg = (const real_T**) p;
            p += config.sz_arg;
            real_T** res = (real_T**) p;
            w = ssGetRWork(S);
            iw = (casadi_int*) ssGetIWork(S);

            {prefix}set_work(m, arg, res, iw, w);
            
            /* Point to input and output buffers */
            int n_p = {prefix}get_id_count(m, "p"); 
            for (i=0; i<n_p;++i) {{
              {prefix}get_id_from_index(m, "p", i, &id);
              {prefix}set(m, "p", id, IMPACT_EVERYWHERE, ssGetInputPortSignal(S,i), IMPACT_FULL);
            }}
            i = n_p;
            {prefix}set(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetInputPortSignal(S,i++), IMPACT_FULL);""")
      if self.nz>0:
        out.write(f"""
            {prefix}set(m, "z_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetInputPortSignal(S,i++), IMPACT_FULL);""")

      out.write(f"""
            {prefix}set(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetInputPortSignal(S,i++), IMPACT_FULL);

            {prefix}solve(m);

            int_T ret = impact_solve(m);
            impact_print_problem(m);
            if (ret && {int(not ignore_errors)}) {{
                static char msg[{200+len(s_function_name)}];
                sprintf(msg, "SFunction '{s_function_name}' failed to compute (error code %d) at t=%.6fs. "
                "Export with ('ignore_errors', true) if you want the simulation to continue anyway.", ret, ssGetT(S));
                ssSetLocalErrorStatus(S, msg);
            }}

            {prefix}print_problem(m);

            i = 0;
            {prefix}get(m, "x_opt", IMPACT_ALL, {'1' if short_output else 'IMPACT_EVERYWHERE'}, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);""")
      if self.nz>0:
        out.write(f"""
            {prefix}get(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);""")

      out.write(f"""
            {prefix}get(m, "u_opt", IMPACT_ALL, {'0' if short_output else 'IMPACT_EVERYWHERE'}, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);

            """)
      if ignore_errors:
        out.write(f"""ssGetOutputPortRealSignal(S, i++)[0] = ret;
        """)

      out.write(f"""

        }}

        static void mdlStart(SimStruct *S) {{
#ifdef  MATLAB_MEX_FILE
          mexPrintf("mdlStart\\n");
#endif
        }}

        static void mdlTerminate(SimStruct *S) {{
#ifdef  MATLAB_MEX_FILE
          mexPrintf("mdlTerminate\\n");
#endif
          cleanup();
        }}


        #ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
        #include "simulink.c"      /* MEX-file interface mechanism */
        #else
        #include "cg_sfun.h"       /* Code generation registration function */
        #endif
        """)

    # Export helper functions
    added_functions = []
    for fun in self._added_functions:
      added_functions.append(fun2s_function(fun, dir=build_dir_abs,ignore_errors=ignore_errors))

    m_build_file_name = os.path.join(build_dir_abs,"build.m")
    with open(m_build_file_name,"w") as out:
      out.write(f"""
        s_function_file_name_base = '{s_function_file_name_base}';
        c_file_name_base = '{c_file_name_base}';
        casadi_codegen_file_name_base = '{casadi_codegen_file_name_base}';
        [build_dir,~,~] = fileparts(mfilename('fullpath'));
        flags={{['-I' build_dir] ['-L' build_dir] ['-I' casadi.GlobalOptions.getCasadiIncludePath()] ['-L' casadi.GlobalOptions.getCasadiPath()]}};
        files = {{[build_dir filesep s_function_file_name_base], [build_dir filesep c_file_name_base]}};
        """)
      if use_codegen:
        out.write(f"""
          files=[files {{ [build_dir filesep casadi_codegen_file_name_base]}}];
          flags=[flags {{['-I' casadi.GlobalOptions.getCasadiIncludePath()] ['-I' casadi.GlobalOptions.getCasadiPath()] '-losqp'}}];
          """)
      else:
        out.write(f"""
          flags=[flags {{'-lcasadi'}}];
          """)
      out.write(f"""
      if ispc
      else
        flags=[flags {{'-g' ['LDFLAGS=\"\$LDFLAGS -Wl,-rpath,' casadi.GlobalOptions.getCasadiPath() ' -Wl,-rpath,' build_dir '\"']}}];
      end
      mex(flags{{:}},files{{:}});\n""")
      for f_name in added_functions:
        out.write(f"mex(flags{{:}}, [build_dir filesep '{f_name}']);\n")

    shutil.copy(os.path.join(self.basename,"templates","python","impact.py"), os.path.join(build_dir_abs,"impact.py"))
    py_file_name = os.path.join(build_dir_abs,"hello_world_" + name+".py")

      
    with open(py_file_name,"w") as out:
      out.write(f"""

import pylab as plt
import numpy as np

from impact import Impact

impact = Impact("{name}",src_dir="..")
""")

      if hello_p_normal_name:
        out.write(f"""
# Example: how to set a parameter
val_{hello_p_normal_name} = [{strlist(hello_p_normal_nominal)}]
impact.set("p", "{hello_p_normal_name}", impact.EVERYWHERE, impact.FULL, val_{hello_p_normal_name})
        """)
      out.write(f"""
print("Solve a single OCP (default parameters)")
impact.solve()

# Get {self.states[0].name()} solution trajectory
print(impact.get("x_opt", "{self.states[0].name()}", impact.EVERYWHERE, impact.FULL))

# Get solution trajectory
x_opt = impact.get("x_opt", impact.ALL, impact.EVERYWHERE, impact.FULL)

# Plotting


_, ax = plt.subplots(2,1,sharex=True)
ax[0].plot(x_opt.T)
ax[0].set_title('Single OCP')
ax[0].set_xlabel('Sample')

print("Running MPC simulation loop")

history = []
for i in range(100):
  impact.solve()

  # Optimal input at k=0
  u = impact.get("u_opt", impact.ALL, 0, impact.FULL)

  # Simulate 1 step forward in time
  # (TODO: use simulation model other than MPC model)
  x_sim = impact.get("x_opt", impact.ALL, 1, impact.FULL)

  # Update current state
  impact.set("x_current", impact.ALL, 0, impact.FULL, x_sim)
  history.append(x_sim)

# More plotting
ax[1].plot(np.hstack(history).T)
ax[1].set_title('Simulated MPC')
ax[1].set_xlabel('Sample')
plt.show()



      """)

    simulink_library_name = "library_"+name
    simulink_library_dirname = os.path.join(build_dir_abs,simulink_library_name)
    simulink_library_filename = simulink_library_dirname+".slx"

    template = os.path.join(self.basename,"templates","simulink","lib2016b.slx")
    diag = Diagram(template,simulink_library_filename,simulink_library_dirname=simulink_library_dirname)

    port_labels_in = []
    for p in parameters_symbols:
      port_labels_in.append(p.name())
    port_labels_in.append('x_initial_guess')

    if self.nz>0:
      port_labels_in.append('z_initial_guess')
    port_labels_in.append('u_initial_guess')
    mask_name = f"IMPACT MPC\\n{name}"
    port_labels_out = []
    port_labels_out.append("x_opt @ k=1" if short_output else "x_opt")
    if self.nz>0:
      port_labels_out.append("z_opt")
    port_labels_out.append("u_opt @ k=0" if short_output else "u_opt")
    if ignore_errors:
      port_labels_out.append("status code (0=good)")
    port_in = []
    for p in parameters_symbols:
      port_in.append(f"NaN({p.shape[0]},{p.shape[1]})")
    mask = Mask(port_labels_in=port_labels_in, port_labels_out=port_labels_out,port_in=port_in,name=mask_name,s_function=s_function_name,dependencies=[name+"_codegen", name])
    diag.add(mask)
    for name,fun in zip(added_functions,self._added_functions):
      mask = Mask(port_labels_in=fun.name_in(), port_labels_out=fun.name_out(), name=fun.name(), s_function=name[:-2])
      diag.add(mask)
    diag.write()


    make_file_name = os.path.join(build_dir_abs,"Makefile")
      
    with open(make_file_name,"w") as out:

      flags = ["-pedantic","-Wall","-Wextra","-Wno-unknown-pragmas","-Wno-long-long","-Wno-unused-parameter","-Wno-unused-const-variable","-Wno-sign-compare","-Wno-unused-but-set-variable","-Wno-unused-variable","-Wno-endif-labels"]
      deps = ["-L"+build_dir_abs,"-Wl,-rpath="+build_dir_abs,"-L"+GlobalOptions.getCasadiPath(),"-I"+GlobalOptions.getCasadiIncludePath(),"-Wl,-rpath="+GlobalOptions.getCasadiPath()]
      if use_codegen:
        lib_codegen_file_name = os.path.join(build_dir_abs,"lib" + name + "_codegen.so")
        lib_codegen_compile_commands = ["gcc","-g","-fPIC","-shared",casadi_codegen_file_name,"-lm","-o"+lib_codegen_file_name]+deps
        deps += ["-l"+name+"_codegen","-losqp"] 
      else:
        deps += ["-lcasadi"]
      lib_compile_commands = ["gcc","-g","-fPIC","-shared",c_file_name,"-o"+lib_file_name]+deps+flags

      hello_compile_commands = ["gcc","-g",hello_c_file_name,"-I"+build_dir_abs,"-l"+lib_name,"-o",hello_file_name]+deps+flags

      if os.name!="nt":
        print("Compiling")
        import subprocess
        if use_codegen:
          p = subprocess.run(lib_codegen_compile_commands, capture_output=True, text=True)
          print(" ".join(p.args))
          if p.returncode!=0:
            raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))

        p = subprocess.run(lib_compile_commands, capture_output=True, text=True)
        print(" ".join(p.args))
        if p.returncode!=0:
          raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))
        # #breaks matlab
        if context!="matlab":
          p = subprocess.run(hello_compile_commands, capture_output=True, text=True)
          if p.returncode!=0:
            raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))

      print(hello_compile_commands)
      if use_codegen:
        out.write(os.path.basename(lib_codegen_file_name) + ": " + os.path.basename(casadi_codegen_file_name) + "\n")
        out.write("\t"+" ".join(lib_codegen_compile_commands)+"\n\n")

      out.write(os.path.basename(lib_file_name) + ": " + os.path.basename(c_file_name) + "\n")
      out.write("\t"+" ".join(lib_compile_commands)+"\n\n")

      out.write(os.path.basename(hello_file_name) + ": " + os.path.basename(hello_c_file_name) + "\n")
      out.write("\t"+" ".join(hello_compile_commands)+"\n\n")

    print("success")
