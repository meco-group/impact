"""MPC class of Impact."""
from re import X
from rockit import Ocp, rockit_pickle_context, rockit_unpickle_context
from casadi import Function, MX, vcat, vvcat, veccat, GlobalOptions, vec, CodeGenerator, DaeBuilder, vertsplit, DM
from rockit.casadi_helpers import prepare_build_dir
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
import copy
from packaging import version

from .dotdict import DotDict, Structure
from .model import Model

class Field:
  def __init__(self,name,type):
    self.name = name
    self.type = type

class Struct:
  def __init__(self,name,fields):
    self.name = name
    self.fields = fields

def modern_casadi():
  if "-" in casadi.__version__:
    return True
  return version.parse(casadi.__version__)>=version.parse("3.6.5")


fields = [Field("sqp_stop_crit","int"),
          Field("n_sqp_iter","int"),
          Field("n_ls","int"),
          Field("n_max_ls","int"),
          Field("n_qp_iter","int"),
          Field("n_max_qp_iter","int"),
          Field("runtime","double")]

solver_stats_type = Struct("solver_stats_bus",fields)

solver_stats_c_type = Struct("solver_stats",fields)

def escape(e):
  return e.replace('\\','/')
  
def field_type_to_DataType(t):
  if t=="double":
    return "double"
  elif t=="float":
    return "single"
  elif t=="int":
    return "int32"

def field_type_to_matlab_c(t):
  if t=="double":
    return "double"
  elif t=="float":
    return "float"
  elif t=="int":
    return "int32_T"

def field_type_to_c(t):
  return t

def write_struct(s,target="c"):
  # #include "tmwtypes.h"
  ret = ""
  ret += "typedef struct {\n"
  conv = field_type_to_c if target=="c" else field_type_to_matlab_c
  for f in s.fields:
    ret+= "  " + conv(f.type) + " " + f.name + ";\n"
  ret += "}" + s.name + ";"
  return ret

def write_bus(s):
  ret = f"{s.name} = Simulink.Bus;\n"
  ret += "elements = {};\n"
  for f in s.fields:
    ret+= "e = Simulink.BusElement;\n"
    ret+= f"e.Name = '{f.name}';\n"
    ret+= f"e.DataType = '{field_type_to_DataType(f.type)}';\n"
    ret+= "elements{end+1} = e;\n"
  ret += f"{s.name}.Elements = [elements{{:}}];"
  return ret

dae_keys = {"x": "differential_states", "z": "algebraic_states", "p": "parameters", "u": "controls"}
dae_rockit_normal = {"x": "state", "z": "algebraic", "p": "parameter", "u": "control"}
dae_rockit_sys_id = {"x": "state", "z": "algebraic", "p": "variable", "u": "parameter"}


# keywords = {"x","u","z","p","c","y"}
# for k in set(keywords):
#   keywords.add("n"+k)
# keywords.add("all")


# def remove_prefix(n,prefix):
#   if n.startswith(prefix):
#     return n[len(prefix):]
#   else:
#     n

# class DotDict(object):
#   """like a dict but access through . """
#   def __init__(self, d=None):
#     if d is None:
#       d = {}
#     self._d = d

#   def __getattr__(self, k):
#     try:
#       return self._d[k]
#     except:
#       raise AttributeError()
  
#   def _update(self, d, allow_keyword=False):
#     for k,v in d.items():
#       if allow_keyword:
#         if k in self._d:
#           old = self._d[k]
#           if isinstance(old,list):
#             self._d[k]=old+v
#           else:
#             self._d[k] = veccat(old,v)
#         else:
#           self._d[k] = v
#       else:
#         if k in keywords:
#           raise Exception("'%s' is a reserved keyword. Please use another name." % k)
#         if k in self._d:
#           raise Exception("Name collision: '%s' already in use." % k)
#         self._d[k] = v
      
#   def __repr__(self,indent=0):
#     s = "{\n"
#     for k,v in sorted(self._d.items()):
#       s += ("  " * (indent+1)) + k
#       try:
#         s += ": " + v.__repr__(indent=indent+1)
#       except:
#         if isinstance(v, list):
#           s+= ": " + str(v)
#         #s += str(v)
#         pass

#       s += "\n"
#     s+=("  " * indent) + "}"
#     return s


# class Structure:
#   def __init__(self, definition,prefix=""):
#     if isinstance(definition,MX):
#       self._symbols = [definition]
#       self._concat = definition
#       self._numel = definition
#       self._names = [definition.name()]
#     else:
#       symbols = []
#       for d in definition:
#         size = d["size"] if "size" in d else 1
#         name = d["name"]
#         symbols.append(MX.sym(prefix+name, size))
#       self._names = [d["name"] for d in definition]
#       self._symbols = symbols
#       self._concat = vcat(symbols)
#       self._numel = self._concat.numel()

#   def __MX__(self):
#     return self._concat


# class Model(DotDict):
#   """This should be a description of the Model class."""

#   def __init__(self, prefix=""):
#     DotDict.__init__(self)
#     self._prefix = prefix
#     self.all = DotDict()

#   def _register(self,key,parts):
#     if isinstance(parts,dict):
#       for k,v in parts.items():
#         self._update({k: v})
#       ks = list(sorted(parts.keys()))
#       self.all._update({key: [parts[k] for k in ks]},allow_keyword=True)
#       self._update({key: vvcat([parts[k] for k in ks])},allow_keyword=True)
#     else:
#       s = Structure(parts,prefix=self._prefix)
#       for e in s._symbols:
#         self._update({remove_prefix(e.name(),self._prefix): e},allow_keyword=isinstance(parts,MX))
#       self.all._update({key: s._symbols},allow_keyword=True)
#       if not isinstance(parts,MX):
#         self._update({key : s._concat},allow_keyword=True)
#       self._update({'n'+key: s._numel},allow_keyword=True)
#       return s

def fun2s_function(fun, name=None, dir=".",ignore_errors=False,build_dir_abs=None):
    fun_name = fun.name()
    if name is None:
      name = fun_name
    s_function_name = name+"_s_function_level2"
    cg_options = {"casadi_real": "real_T", "casadi_int": "int_T"}
    cg = CodeGenerator(name, cg_options)
    cg.add(fun)
    code = cg.dump()
    use_codegen = True
    if "#error" in code:
      use_codegen = False
    print(f"Use codegen for {fun_name}? {use_codegen}")
    if not use_codegen:
      code = f"""#include <casadi/casadi_c.h>
      #define casadi_int long long int
      """
    for i in range(fun.n_in()):
      if not fun.sparsity_in(i).is_dense():
        raise Exception("Sparse inputs not supported")
    for i in range(fun.n_out()):
      if not fun.sparsity_out(i).is_dense():
        raise Exception("Sparse outputs not supported")

    s_function_file_name_base = s_function_name+".c"
    s_function_file_name = os.path.join(dir,s_function_file_name_base)

    def c_api(suffix):
      if use_codegen:
        if suffix=="eval":
          return fun_name+"("
        return fun_name+"_"+suffix+"("
      else:
        return "casadi_c_"+suffix+"_id(id" + ("," if suffix in ["eval","work","sparsity_in","sparsity_out","release"] else "")

    with open(s_function_file_name,"w") as out:
      out.write(f"""
        #define S_FUNCTION_NAME {s_function_name}
        #define S_FUNCTION_LEVEL 2

        #include "simstruc.h"
        {code}

        static int_T n_in, n_out;
        static casadi_int sz_arg, sz_res, sz_iw, sz_w;
        static int id;

        static void mdlInitializeSizes(SimStruct *S) {{
          int flag;
          char casadi_file[2048];
          ssSetNumSFcnParams(S, 0);
          if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {{
              return; /* Parameter mismatch will be reported by Simulink */
          }}
        """)

      if not use_codegen:
        out.write(f"""
          id = casadi_c_id("{fun_name}");
          if (id<0) {{
            flag = casadi_c_push_file("{escape(build_dir_abs)}/{name}.casadi");
            if (flag) {{
  #ifdef MATLAB_MEX_FILE
                mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                                  "Failed to initialize.");
  #else
                return;
  #endif     
            }}
            id = casadi_c_id("{fun_name}");
            if (id<0) {{
  #ifdef MATLAB_MEX_FILE
                mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                                  "Could not locate function.");
  #else
                return;
  #endif     
            }}
          }}
        """)

      out.write(f"""
        /* Read in CasADi function dimensions */
        n_in  = {c_api("n_in")});
        n_out = {c_api("n_out")});
        {c_api("work")}&sz_arg, &sz_res, &sz_iw, &sz_w);

        /* Set up simulink input/output ports */
        int_T i;
        if (!ssSetNumInputPorts(S, n_in)) return;
        for (i=0;i<n_in;++i) {{
          const casadi_int* sp = {c_api("sparsity_in")}i);
          /* Dense inputs assumed here */
          ssSetInputPortDirectFeedThrough(S, i, 1);
          ssSetInputPortMatrixDimensions(S, i, sp[0], sp[1]);
          ssSetInputPortRequiredContiguous(S, i, 1);
        }}

        if (!ssSetNumOutputPorts(S, n_out)) return;
        for (i=0;i<n_out;++i) {{
          const casadi_int* sp = {c_api("sparsity_out")}i);
          /* Dense outputs assumed here */
          ssSetOutputPortMatrixDimensions(S, i, sp[0], sp[1]);
        }}

        ssSetNumSampleTimes(S, 1);
        
        /* Set up CasADi function work vector sizes */
        ssSetNumRWork(S, sz_w);
        ssSetNumIWork(S, 2*sz_iw);
        ssSetNumPWork(S, sz_arg+sz_res);
        ssSetNumNonsampledZCs(S, 0);

        /* specify the sim state compliance to be same as a built-in block */
        ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

        ssSetOptions(S,
                    SS_OPTION_EXCEPTION_FREE_CODE |
                    SS_OPTION_USE_TLC_WITH_ACCELERATOR);

        /* Signal that we want to use the CasADi Function */
        {c_api("incref")});
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

          int mem = {c_api("checkout")});

          /* Run the CasADi function */
          int_T ret = {c_api("eval")}arg,res,iw,w,mem);

          if (ret && {int(not ignore_errors)}) {{
              static char msg[{100+len(s_function_name)}];
              sprintf(msg, "SFunction '{s_function_name}' failed to compute (error code %d) at t=%.6fs.", ret, ssGetT(S));
              #ifdef ssSetLocalErrorStatus
                ssSetLocalErrorStatus(S, msg);
              #else
                #ifdef ssSetErrorStatus
                  ssSetErrorStatus(S, msg);
                #endif
              #endif
          }}

          /* Release hold */
          {c_api("release")}mem);

        }}

        static void mdlTerminate(SimStruct *S) {{
          {c_api("decref")});
          """)

      if not use_codegen:
          out.write(f"""
            //casadi_c_pop();
          """)

      out.write(f"""}}

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
  def __init__(self,s_function, port_labels_in=None, port_labels_out=None, block_name="", name=None, port_in=None,port_out=None,dependencies=None,init_code=""):
    self.dependencies = dependencies
    self.s_function = s_function
    self.port_labels_in = port_labels_in
    self.port_labels_out = port_labels_out
    if port_in is None:
      port_in = []
    self.port_in = port_in
    self.init_code = init_code
    self.name = name
    self.block_name = block_name
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
    <Block BlockType="S-Function" Name="Block_{self.block_name}" SID="{self.base_id}">
      <P Name="Ports">[{len(self.port_labels_in)}, {len(self.port_labels_out)}]</P>
      <P Name="Position">[{sfun_margin_left}, {sfun_margin_top}, {sfun_margin_left+self.sfun_width}, {sfun_margin_top+self.sfun_height}]</P>
      <P Name="ZOrder">{self.zorder}</P>
      <P Name="FunctionName">{self.s_function}</P>
      <P Name="SFunctionDeploymentMode">off</P>
      <P Name="EnableBusSupport">off</P>
      <P Name="SFcnIsStateOwnerBlock">off</P>
      <P Name="InitFcn">{self.init_code}</P>
      <Object PropName="MaskObject" ObjectID="{self.base_id+7}" ClassName="Simulink.Mask">
        <P Name="Display" Class="char">{self.mask_commands}</P>
      </Object>"""
    if self.dependencies:
      s+=f"""<P Name="SFunctionModules">&apos;{" ".join(self.dependencies)}&apos;</P>"""
    s += f"</Block>"
    try:
      system.append(etree.fromstring(s))
    except Exception as e:
      for i,line in enumerate(s.split("\n")):
        print(i,line)
      raise e

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
  """This should be a description of the MPC class.
  It's common for programmers to give a code example inside of their
  docstring::

      from impact import MPC
      mpc = MPC(T=2.0)

  Here is a link to :py:meth:`__init__`.
  """

  def __init__(self, **kwargs):
    """Inits MPC class."""

    Ocp.__init__(self, **kwargs)
    self._expr = Model()
    self.basename = os.path.dirname(__file__)
    self._added_functions = []

  @property
  def expr(self):
      return self._expr

  def control(self,*args,**kwargs):
    """Defines control variable"""

    if len(args)>0 and isinstance(args[0],str):
        name = args[0]
        args = args[1:]
    else:
      name = "u%d" % self.nu
    u = MX.sym(name,*args)
    self.register_control(u,**kwargs)
    self.expr._register('u', {name: u})
    return u

  def state(self,*args,**kwargs):
    if len(args)>0 and isinstance(args[0],str):
        name = args[0]
        args = args[1:]
    else:
      if "quad" in kwargs and kwargs["quad"]:
        name = "xq%d" % self.nxq
      else:
        name = "x%d" % self.nx
    x = MX.sym(name,*args)
    self.register_state(x,**kwargs)
    self.expr._register('x', {name: x})
    return x

  def parameter(self,*args,**kwargs):
    if len(args)>0 and isinstance(args[0],str):
        name = args[0]
        args = args[1:]
    else:
      name = "p%d" % self.np
    p = MX.sym(name,*args)
    self.register_parameter(p,**kwargs)
    self.expr._register('p', {name: p})
    return p

  def variable(self,*args,**kwargs):
    if len(args)>0 and isinstance(args[0],str):
        name = args[0]
        args = args[1:]
    else:
      name = "v%d" % self.nv
    v = MX.sym(name,*args)
    self.register_variable(v,**kwargs)
    self.expr._register('v', {name: v})
    return v

  def add_function(self, fun):
    self._added_functions.append(fun)

  def add_simulink_fmu(self,name,verbose=True):
    """
      Not supported: 
      * time dependence
      * delays
      Perhpas SS is better
      Caveats:
      * scaling for finite diff
    """
    from pathlib import Path
    import zipfile
    fmu_path = Path(name).resolve()
    unzipped_path =  fmu_path.parent / fmu_path.stem
    with zipfile.ZipFile(fmu_path, 'r') as zip_ref:
        zip_ref.extractall(unzipped_path)

    scalar_variables = []
    with open(unzipped_path/ 'modelDescription.xml','r') as modelDescription_file:
      tree = etree.parse(modelDescription_file)
      for var in tree.find('.//ModelVariables').findall('ScalarVariable'):
        scalar_variables.append(dict(var.items()))
    
    root = tree.find('.')
    modelName = root.attrib["modelName"]
    guid = root.attrib["guid"]
    resource = "file://" + str(unzipped_path)
    
    dll_name = unzipped_path / "binaries" / "linux64" / "vdp"

    u_i = [int(e["valueReference"]) for e in scalar_variables if e["causality"]=="input"]
    y_i = [int(e["valueReference"]) for e in scalar_variables if e["causality"]=="output"]
    x_i = [int(e["valueReference"]) for e in scalar_variables if e["causality"]=="parameter" and e["description"].startswith("x")]
    assert len(y_i)==len(x_i)
    p_i = [int(e["valueReference"]) for e in scalar_variables if e["causality"]=="parameter" and not e["description"].startswith("x")]
    print(x_i,y_i)
    
    nu = len(u_i)
    nx = len(x_i)
    np = len(p_i)
    print(nx)

    def intlist(L):
      return "{" + ", ".join(str(e) for e in L) + "}"
    with open("fmi_wrapper.cpp","w") as fmi_wrapper:
      fmi_wrapper.write(f"""
#include <math.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <exception>
#include <stdarg.h>

#ifdef _WIN32 // also for 64-bit
#include <windows.h>
#else // _WIN32
#include <dlfcn.h>
#endif // _WIN32

#include <fmi2/fmi2Functions.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

std::string system_infix() {{
#if defined(_WIN32)
  // Windows system
#ifdef _WIN64
  return "win64";
#else
  return "win32";
#endif
#elif defined(__APPLE__)
  // OSX
  return sizeof(void*) == 4 ? "darwin32" : "darwin64";
#else
  // Linux
  return sizeof(void*) == 4 ? "linux32" : "linux64";
#endif
}}

std::string dll_suffix() {{
#if defined(_WIN32)
  // Windows system
  return ".dll";
#elif defined(__APPLE__)
  // OSX
  return ".dylib";
#else
  // Linux
  return ".so";
#endif
}}


/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif


std::vector<casadi_int> dense_vec(int n) {{
    std::vector<casadi_int> ret = {{n , 1, 0, n}};
    for (int i=0;i<n;++i) {{
        ret.push_back(i);
    }}
    return ret;
}}

template<typename T>
T* get_ptr(std::vector<T> &v) {{
if (v.empty())
  return nullptr;
else
  return &v.front();
}}


#if defined(_WIN32) // also for 64-bit
    typedef HINSTANCE handle_t;
#else
    typedef void* handle_t;
#endif

  /// String representation, any type
  template<typename T>
  std::string str(const T& v);

  // Implementations
  template<typename T>
  std::string str(const T& v) {{
    std::stringstream ss;
    ss << v;
    return ss.str();
  }}

  handle_t init_handle(const std::string& name) {{
    handle_t ret;
#ifdef _WIN32
    ret = LoadLibrary(TEXT(name.c_str()));
    if (ret==0) {{
      throw std::system_error(0, std::generic_category(), "CommonExternal: Cannot open \"" + name + "\". "
      "Error code (WIN32): " + str(GetLastError()));
    }}
#else // _WIN32
    ret = dlopen(name.c_str(), RTLD_LAZY);
    if (ret==nullptr) {{
      throw std::system_error(0, std::generic_category(), "CommonExternal: Cannot open \"" + name + "\". "
      "Error code: " + str(dlerror()));
    }}
    // reset error
    dlerror();
#endif // _WIN32
    return ret;
  }}

  void close_handle(handle_t h) {{
    #ifdef _WIN32
        if (h) FreeLibrary(h);
    #else // _WIN32
        if (h) dlclose(h);
    #endif // _WIN32
  }}

  typedef void (*signal_t)(void);

  signal_t get_function(handle_t h, const std::string& sym) {{
#ifdef _WIN32
    return (signal_t)GetProcAddress(h, TEXT(sym.c_str()));
#else // _WIN32
    signal_t fcnPtr = (signal_t)dlsym(h, sym.c_str());
    if (dlerror()) {{
      fcnPtr=nullptr;
      dlerror(); // Reset error flags
    }}
    return fcnPtr;
#endif // _WIN32
  }}

void logger(fmi2ComponentEnvironment componentEnvironment,
    fmi2String instanceName,
    fmi2Status status,
    fmi2String category,
    fmi2String message, ...) {{
  // Variable number of arguments
  va_list args;
  va_start(args, message);
  // Static & dynamic buffers
  char buf[256];
  size_t buf_sz = sizeof(buf);
  char* buf_dyn = nullptr;
  // Try to print with a small buffer
  int n = vsnprintf(buf, buf_sz, message, args);
  // Need a larger buffer?
  if (n > buf_sz) {{
    buf_sz = n + 1;
    buf_dyn = new char[buf_sz];
    n = vsnprintf(buf_dyn, buf_sz, message, args);
  }}
  // Print buffer content
  if (n >= 0) {{
    std::cout << "[" << instanceName << ":" << category << "] "
      << (buf_dyn ? buf_dyn : buf) << std::endl;
  }}
  // Cleanup
  delete[] buf_dyn;
  va_end(args);
  // Throw error if failure
  if (n<=0) {{
    throw std::runtime_error("Print failure while processing '" + std::string(message) + "'");
  }}
}}


void my_assert(bool v, const std::string& msg) {{
  if (!v) throw std::runtime_error(msg);
}}

class FMU {{
  public:
    FMU() {{
      h_ = init_handle("{dll_name}"+dll_suffix());
      instantiate_ = reinterpret_cast<fmi2InstantiateTYPE*>(get_function(h_,"fmi2Instantiate"));
      free_instance_ = reinterpret_cast<fmi2FreeInstanceTYPE*>(get_function(h_,"fmi2FreeInstance"));
      reset_ = reinterpret_cast<fmi2ResetTYPE*>(get_function(h_,"fmi2Reset"));
      setup_experiment_ = reinterpret_cast<fmi2SetupExperimentTYPE*>(
        get_function(h_,"fmi2SetupExperiment"));
      enter_initialization_mode_ = reinterpret_cast<fmi2EnterInitializationModeTYPE*>(
        get_function(h_,"fmi2EnterInitializationMode"));
      exit_initialization_mode_ = reinterpret_cast<fmi2ExitInitializationModeTYPE*>(
        get_function(h_,"fmi2ExitInitializationMode"));
      get_real_ = reinterpret_cast<fmi2GetRealTYPE*>(get_function(h_,"fmi2GetReal"));
      set_real_ = reinterpret_cast<fmi2SetRealTYPE*>(get_function(h_,"fmi2SetReal"));
      get_integer_ = reinterpret_cast<fmi2GetIntegerTYPE*>(get_function(h_,"fmi2GetInteger"));
      set_integer_ = reinterpret_cast<fmi2SetIntegerTYPE*>(get_function(h_,"fmi2SetInteger"));
      get_boolean_ = reinterpret_cast<fmi2GetBooleanTYPE*>(get_function(h_,"fmi2GetBoolean"));
      set_boolean_ = reinterpret_cast<fmi2SetBooleanTYPE*>(get_function(h_,"fmi2SetBoolean"));
      get_string_ = reinterpret_cast<fmi2GetStringTYPE*>(get_function(h_,"fmi2GetString"));
      set_string_ = reinterpret_cast<fmi2SetStringTYPE*>(get_function(h_,"fmi2SetString"));
      do_step_ = reinterpret_cast<fmi2DoStepTYPE*>(get_function(h_,"fmi2DoStep"));

      logging_on_ = {int(verbose)};
      c_.resize(64);

      c_[0] = instantiate();
      setup_experiment(0);

      u_i_ = {intlist(u_i)};
      x_i_ = {intlist(x_i)};
      y_i_ = {intlist(y_i)};
      p_i_ = {intlist(p_i)};

    }}
    ~FMU() {{
      close_handle(h_);
    }}
    fmi2Component instantiate() const {{
      fmi2String instanceName = "{modelName}";
      fmi2String fmuGUID = "{guid}";
      fmi2String fmuResourceLocation = "{resource}";
      fmi2Boolean visible = fmi2False;
      static fmi2CallbackFunctions cbf;
      cbf.logger = logger;
      cbf.allocateMemory = calloc;
      cbf.freeMemory = free;
      cbf.stepFinished = 0;
      cbf.componentEnvironment = 0;
      fmi2Component c = instantiate_(instanceName, fmi2CoSimulation, fmuGUID, fmuResourceLocation,
        &cbf, visible, logging_on_);
      return c;
    }}

    int setup_experiment(int mem) const {{
      // Call fmi2SetupExperiment
      fmi2Status status = setup_experiment_(c_[mem], fmi2False, 0, 0., fmi2True, 1.);
      if (status != fmi2OK) {{
        std::cerr << "fmi2SetupExperiment failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int reset(int mem) const {{
      fmi2Status status = reset_(c_[mem]);
      if (status != fmi2OK) {{
        std::cerr << "fmi2Reset failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int enter_initialization_mode(int mem) const {{
      fmi2Status status = enter_initialization_mode_(c_[mem]);
      if (status != fmi2OK) {{
        std::cerr << "fmi2EnterInitializationMode failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int exit_initialization_mode(int mem) const {{
      fmi2Status status = exit_initialization_mode_(c_[mem]);
      if (status != fmi2OK) {{
        std::cerr << "fmi2ExitInitializationMode failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int set_x(int mem, const double* x) {{
      fmi2Status status = set_real_(c_[mem], get_ptr(x_i_), x_i_.size(), x);
      if (status != fmi2OK) {{
        std::cerr << "fmi2SetReal x failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}
    int set_u(int mem, const double* u) {{
      fmi2Status status = set_real_(c_[mem], get_ptr(u_i_), u_i_.size(), u);
      if (status != fmi2OK) {{
        std::cerr << "fmi2SetReal u failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}
    int set_p(int mem, const double* p) {{
      fmi2Status status = set_real_(c_[mem], get_ptr(p_i_), p_i_.size(), p);
      if (status != fmi2OK) {{
        std::cerr << "fmi2SetReal p failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int get_x(int mem, double* x) {{
      fmi2Status status = get_real_(c_[mem], get_ptr(y_i_), y_i_.size(), x);
      if (status != fmi2OK) {{
        std::cerr << "fmi2GetReal x failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}

    int do_step(int mem, double dt) {{
      fmi2Status status = do_step_(c_[mem], 0, dt, fmi2False);
      if (status != fmi2OK) {{
        std::cerr << "fmi2DosStep x failed: " << status << std::endl;
        return 1;
      }}
      return 0;
    }}
  
  private:
    handle_t h_;
    fmi2InstantiateTYPE* instantiate_;
    fmi2FreeInstanceTYPE* free_instance_;
    fmi2ResetTYPE* reset_;
    fmi2SetupExperimentTYPE* setup_experiment_;
    fmi2EnterInitializationModeTYPE* enter_initialization_mode_;
    fmi2ExitInitializationModeTYPE* exit_initialization_mode_;
    fmi2GetRealTYPE* get_real_;
    fmi2SetRealTYPE* set_real_;
    fmi2GetBooleanTYPE* get_boolean_;
    fmi2SetBooleanTYPE* set_boolean_;
    fmi2GetIntegerTYPE* get_integer_;
    fmi2SetIntegerTYPE* set_integer_;
    fmi2GetStringTYPE* get_string_;
    fmi2SetStringTYPE* set_string_;
    fmi2GetDirectionalDerivativeTYPE* get_directional_derivative_;
    fmi2DoStepTYPE* do_step_;

    std::vector<fmi2Component> c_;
    std::vector<fmi2ValueReference> x_i_;
    std::vector<fmi2ValueReference> y_i_;
    std::vector<fmi2ValueReference> u_i_;
    std::vector<fmi2ValueReference> p_i_;

    bool logging_on_;
}};

static FMU fmu;

#ifdef __cplusplus
extern "C" {{
#endif


CASADI_SYMBOL_EXPORT int F(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){{
  // do_step seems to do an univertible operation on time -> reset+setup_experiment
  if (fmu.setup_experiment(mem)) return 1;
  if (fmu.enter_initialization_mode(mem)) return 1;
  if (fmu.set_x(mem, arg[0])) return 1;
  if (fmu.set_u(mem, arg[1])) return 1;
  if (fmu.set_p(mem, arg[2])) return 1;
  if (fmu.exit_initialization_mode(mem)) return 1;
  if (fmu.do_step(mem, arg[4][0])) return 1;
  if (fmu.get_x(mem, res[0])) return 1;
  if (fmu.reset(mem)) return 1;
  return 0;
}}

CASADI_SYMBOL_EXPORT int F_alloc_mem(void) {{
  return 0;
}}

CASADI_SYMBOL_EXPORT int F_init_mem(int mem) {{
  std::cout << "test init" << std::endl;
  return 0;
}}

CASADI_SYMBOL_EXPORT void F_free_mem(int mem) {{
}}

CASADI_SYMBOL_EXPORT int F_checkout(void) {{
  return 0;
}}

CASADI_SYMBOL_EXPORT void F_release(int mem) {{
}}



CASADI_SYMBOL_EXPORT void F_incref(void) {{
}}

CASADI_SYMBOL_EXPORT void F_decref(void) {{
}}

CASADI_SYMBOL_EXPORT casadi_int F_n_in(void) {{ return 5;}}

CASADI_SYMBOL_EXPORT casadi_int F_n_out(void) {{ return 1;}}

CASADI_SYMBOL_EXPORT casadi_real F_default_in(casadi_int i) {{
  switch (i) {{
    default: return 0;
  }}
}}

CASADI_SYMBOL_EXPORT const char* F_name_in(casadi_int i) {{
  switch (i) {{
    case 0: return "x";
    case 1: return "u";
    case 2: return "p";
    case 3: return "t";
    case 4: return "dt";
    default: return 0;
  }}
}}

CASADI_SYMBOL_EXPORT const char* F_name_out(casadi_int i) {{
  switch (i) {{
    case 0: return "xp";
    default: return 0;
  }}
}}

CASADI_SYMBOL_EXPORT const casadi_int* F_sparsity_in(casadi_int i) {{
  switch (i) {{
    case 0: 
      {{
        static std::vector<casadi_int> sp_x = dense_vec({nx});
        return get_ptr(sp_x);
      }}
      break;
    case 1:
      {{
        static std::vector<casadi_int> sp_u = dense_vec({nu});
        return get_ptr(sp_u);
      }}
      break;
    case 2:
      {{
        static std::vector<casadi_int> sp_p = dense_vec({np});
        return get_ptr(sp_p);
      }}
      break;
    case 3:
    case 4:
      {{
        static std::vector<casadi_int> sp_t = dense_vec(1);
        return get_ptr(sp_t);
      }}
      break;
    default: return 0;
  }}
}}

CASADI_SYMBOL_EXPORT const casadi_int* F_sparsity_out(casadi_int i) {{
  switch (i) {{
    case 0: 
      {{
        static std::vector<casadi_int> sp_x = dense_vec({nx});
        return get_ptr(sp_x);
      }}
      break;
    default: return 0;
  }}
}}


#ifdef __cplusplus
}} /* extern "C" */
#endif
      """)

    import subprocess
    from pathlib import Path
    cmd = f"g++ fmi_wrapper.cpp -std=c++11 -I{Path(__file__).parent.parent / 'interfaces'} -shared -fPIC -o fmi_wrapper.so"
    print(cmd)
    subprocess.run(cmd, shell=True)
    F = casadi.external("F", "fmi_wrapper.so",{"enable_fd":True})
    print(F)

    x = self.state(nx)
    u = self.control(nu)
    p = self.parameter("fmu_p",np)

    self.set_next(x, F(x, u, p, self.t, 0.1))

    return F

  def add_model(self,name,file_name,mode='normal'):
    """Creates a model based on a yaml file

        :param name: Name of the model
        :type name: string

        :param file_name: Path to the yaml file
        :type name: string

        :param mode: normal or sys_id - in sys_id, parameters are promoted to variables and controls become parameters
        :type mode: string

        :return: model
        :rtype: Model

    """

    if mode=="sys_id":
      dae_rockit = dae_rockit_sys_id
    elif mode=="normal":
      dae_rockit = dae_rockit_normal
    else:
      raise Exception("Unknown mode: '%s' - pick 'normal' or 'sys_id'" % mode)

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

      model_file_name = os.path.expanduser(external["file_name"])
      if not os.path.isabs(model_file_name):
        model_file_name = os.path.join(os.path.dirname(file_name),model_file_name)
      if external["type"]=="casadi_serialized":
        model = Function.load(model_file_name)
      elif external["type"]=="fmu":
        unzipped_path = name+"_unzipped"

        import shutil
        #if os.path.isdir(unzipped_path): shutil.rmtree(unzipped_path)  
        # Unzip
        import zipfile
        #with zipfile.ZipFile(model_file_name, 'r') as zip_ref: zip_ref.extractall(unzipped_path)

        dae = DaeBuilder(name, unzipped_path)

        model = dae.create('f', ['x', 'u', 'p'], ['ode','ydef'])
      else:
        raise Exception("Unknown external type: %s" % external["type"])
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
      rockit_name = dae_rockit[key] if key in dae_rockit else None
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
          if rockit_name:
            register = getattr(self,"register_"+rockit_name)
            for e in v.primitives():
              register(e)
        else:
          if model:
            v = getattr(self,rockit_name)("dummy",var_len)
          else:
            v = MX(0, 1)
        args[key] = v
      nd[key] = var_len

    if model:
      if "ydef" in model.name_out():
        var_len = model.numel_out("ydef")
      else:
        var_len = 0
      if var_len>0:
        assert "outputs" in model_meta
        names["y"] = [e['name'] for e in model_meta["outputs"]]
      else:
        names["y"] = []
    else:
      if "outputs" in model_meta:
        names["y"] = [e['name'] for e in model_meta["outputs"]]
      else:
        names["y"] = []

    if 'parameters' in model_meta:
      # Retrieve parameter values
      for p in model_meta['parameters']:
        self.set_value(getattr(m,p['name']),p['value'])
        m._register('p_val',{p['name']+'_val': DM(p['value'])})

    # Define Outputs
    outputs = {}
    definitions = casadi.__dict__
    locals = dict(m._d)
    for k,v in m._d.items():
      if k.startswith(name+"."):
        locals[k[len(name)+1:]] = v

    if "external" in equations:
      model_res = model(**args)
    
    # Parse inline equations
    if "inline" in equations:
      model_res = {}
      inline = equations["inline"]

      if "outputs" in inline:
        outputs = inline["outputs"]
        outputs_ordered = []
        for local_name in names["y"]:
          if local_name not in outputs:
            raise Exception()
          output = eval(outputs[local_name],casadi.__dict__,locals)
          outputs_ordered.append(output)
          locals[local_name] = output
        model_res["ydef"] = vvcat(outputs_ordered)
        assert len(names["y"])==len(outputs)

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



      model_res["ode"] = casadi.cse(model_res["ode"])

    ode = MX(0,1)
    alg = MX(0,1)
    if nd["x"]:
      assert "ode" in model_res
      ode = model_res["ode"]
      self.set_der(m.x, ode)
    if nd["z"]:
      assert "alg" in model_res
      self.add_alg(model_res["alg"])

    if "ydef" in model_res:
      m._register("y",dict(zip(names["y"],vertsplit(model_res["ydef"]))))
    m._update({"ode": ode, "alg": alg},allow_keyword=True)
    return m

  @staticmethod
  def patch_codegen(name,ocp,qp_error_on_fail=True):
    import re
    with open(name,'r') as fin:
      lines = fin.readlines()

    name_h = name[:-1]+"h"

    with open(name_h,'r') as fin:
      lines_h = fin.readlines()

    increfs = []

    qp = False

    fun_context = ""

    with open(name,'w') as fout:
      for line in lines:

        if line.startswith("/* "):
          m = re.search(r"/\* (\w+):",line)
          if m:
            fun_context = m.group(1)

        if line.startswith("}"):
          fun_context = ""

        if "return 0;" in line and fun_context=="solver":
          s = "end_t = clock();\n"
          s+= "CASADI_PREFIX(stats).runtime = (casadi_real)(end_t - start_t) / CLOCKS_PER_SEC;\n"
          line = s+line

        if "Add prefix to internal symbol" in line:
          line = write_struct(Struct("solver_stats", solver_stats_type.fields))+"\n"+line
        # Bugfix https://gitlab.kuleuven.be/meco/projects/sbo_dirac/dirac_mpc/-/issues/11
        if "return fmax(x, y);" not in line and "CASADI_PREFIX" not in line:
          line = re.sub(r"\bfmax\b","casadi_fmax", line)
        # Bugfix https://github.com/casadi/casadi/issues/2835

        m=re.search(r"\bcasadi_f\d+_checkout\b",line)
        if m:
          increfs.append(m.group(0))

        if "ocpfun_checkout" in line and not modern_casadi():
          fout.write(line)
          line = " ".join([e+"();" for e in increfs])
          
        if "casadi_int A_colind" in line:
          line = line.replace("casadi_int","c_int")

        if "/* Solve the QP */" in line:
          qp = True

        if "if (grampc_driver(arg, res, iw, w, mem)) return 1;" in line:
          line = "mem = grampc_driver_checkout();\n" + line + "grampc_driver_release(mem);\n"
        
        if "/* Detecting indefiniteness */" in line:
          qp = False

        if qp_error_on_fail:
          if ("d.dlam+" in line or "d->dlam+" in line) and "res" in line:
            line = line + "int flag="

          if "Detecting indefiniteness" in line:
            line = "if (flag) return -100;\n" + line

        if "#include <math.h>" in line:
          line = "#include <time.h>\n" + line

        if False and qp and "casadi_f" in line:
          indent = line[:len(line)-len(line.lstrip())]
          line = indent + "if (" + line.strip()[:-1] + ") return 1;\n"

        if "static const casadi_int casadi_s0" in line:
          if "Grampc" in str(ocp._method):
            line = """
        typedef struct {
            int sqp_stop_crit;
            int n_sqp_iter;
            int n_ls;
            int n_max_ls;
            int n_qp_iter;
            int n_max_qp_iter;
            double runtime;
        } compat_solver_stats;
        static solver_stats CASADI_PREFIX(stats);
        const compat_solver_stats* grampc_driver_get_stats(void);
        int grampc_driver_checkout(void);
        void grampc_driver_release(int);

        const solver_stats* ocpfun_stats() { 
          const compat_solver_stats* s = grampc_driver_get_stats();
          CASADI_PREFIX(stats).sqp_stop_crit = s->sqp_stop_crit;
          CASADI_PREFIX(stats).n_sqp_iter = s->n_sqp_iter;
          CASADI_PREFIX(stats).n_qp_iter = s->n_qp_iter;
          CASADI_PREFIX(stats).runtime = s->runtime;
          return &CASADI_PREFIX(stats);
        }
            """ + line
          else:
            line = "static solver_stats CASADI_PREFIX(stats);\n" + \
                  "const solver_stats* ocpfun_stats() { return &CASADI_PREFIX(stats); }\n"+ \
                  line
        
        # if "struct casadi_sqpmethod_prob p;" in line:
          # line = "clock_t start_t, end_t;\nstart_t=clock();\n" + line

        struct_strings = ["struct casadi_sqpmethod_prob p;",
                          "struct casadi_feasiblesqpmethod_prob p;",
                          "struct casadi_ipopt_prob p;",
                          "casadi_fatrop_prob p;"
                          ]

        if any(s in line for s in struct_strings):
          line = "clock_t start_t, end_t;\nstart_t=clock();\n" + line

        if "MAIN OPTIMIZATION LOOP" in line:
          s=""
          for f in solver_stats_type.fields:
            s += f"CASADI_PREFIX(stats).{f.name} = 0;\n"
          line = s + line
        if "Formulate the QP" in line:
          line = "CASADI_PREFIX(stats).n_sqp_iter+=1;" + "\n"+line

        if "iter_count" in line:
          m = re.search(r"if \(iter_count >= (\d+)\) break;",line)
          if m:
            line = f"if (iter_count >= {m.group(1)}) {{ CASADI_PREFIX(stats).sqp_stop_crit = 1; break; }}\n"
          
        if "Candidate accepted, update dual variables" in line:
          line = "CASADI_PREFIX(stats).n_ls += ls_iter;" + "\n"+\
          "if (ls_iter>CASADI_PREFIX(stats).n_max_ls) CASADI_PREFIX(stats).n_max_ls = ls_iter;" + "\n"+line

        if "Get solution" in line:
          line = "CASADI_PREFIX(stats).n_qp_iter += d.iter;" + "\n"+\
          "if (d.iter>CASADI_PREFIX(stats).n_max_qp_iter) CASADI_PREFIX(stats).n_max_qp_iter = d.iter;" + "\n"+line

        fout.write(line)

    with open(name_h,'w') as fout:
      for line in lines_h:
        if "ocpfun(" in line:
          line = write_struct(Struct("solver_stats", solver_stats_type.fields))+"\n"+\
          "const solver_stats* ocpfun_stats();" + "\n" + \
          "\n"+line
        fout.write(line)
      fout.write("")

  def opti_x(self):
    return self._method.opti.x

  def opti_f(self):
    return self._method.opti.f

  def opti_g(self):
    return self._method.opti.g

  def opti_lbg(self):
    return self._method.opti.lbg

  def opti_ubg(self):
    return self._method.opti.ubg

  def opti_lam_g(self):
    return self._method.opti.lam_g

  def opti_p(self):
    return self._method.opti.p


  def export(self,name,src_dir=".",use_codegen=None,context=None,ignore_errors=False,short_output=True,qp_error_on_fail=True,c_flags=["-O3"]):
    build_dir_rel = name+"_build_dir"
    build_dir_abs = os.path.join(os.path.abspath(src_dir),build_dir_rel)


    prepare_build_dir(build_dir_abs)

    artifacts = list(self._method.artifacts)

    for e in artifacts:
      shutil.copy(os.path.join(self._method.build_dir_abs, e.name), build_dir_abs)
    print(self.x)
    print(self.z)
    print(self.u)

    print(self.p)

    [_,states] = self.sample(self.x,grid='control')
    [_,algebraics] = self.sample(self.z,grid='control')
    [_,controls] = self.sample(self.u,grid='control-')
    parameters_symbols = self.parameters['']+self.parameters['control']+self.parameters['control+']
    parameters = []
    for p in self.parameters['']:
      parameters.append(self.value(p))
    for p in self.parameters['control']:
      parameters.append(self.sample(p,grid='control-')[1])
    for p in self.parameters['control+']:
      parameters.append(self.sample(p,grid='control')[1])
    variables = []
    variables_symbols = self.variables['']+self.variables['control']+self.variables['control+']
    for v in self.variables['']:
      variables.append(self.value(v))
    for v in self.variables['control']:
      variables.append(self.sample(v,grid='control-')[1])
    for v in self.variables['control+']:
      variables.append(self.sample(v,grid='control')[1])

    casadi_fun_name = 'ocpfun'
    is_coll = False
    if hasattr(self._method, "Xc"):
      is_coll = True

    lam_g = self._method.opti.lam_g
    parameter_names = [p.name() for p in parameters_symbols]
    variable_names = [v.name() for v in variables_symbols]

    hotstart_symbol = lam_g
    hotstart_symbol = MX(0,1)

    grid = self.sample(self.t, grid='control')[1]

    ocpfun = self.to_function(casadi_fun_name,
      [states]+(["z"] if is_coll else [MX()])+[controls]+[vvcat(variables)]+parameters+[hotstart_symbol],
      [states,algebraics,controls,vvcat(variables),hotstart_symbol, grid],
      ['x0','z0','u0','v0'] + parameter_names + ['hotstart_in'],
      ['x','z','u','v'] + ['hotstart_out']+['grid'])

    casadi_codegen_file_name_base = name+"_codegen.c"
    casadi_codegen_file_name = os.path.join(build_dir_abs,casadi_codegen_file_name_base)


    if not self._state_next:
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
          sim_p.append(p)
      sim_p = vvcat(sim_p)
      if sim_p.numel()>0:
        args.append(sim_p)
        label_in.append("p")
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

    if not self._state_next:
      sys_dae = self.sys_dae()
      sys_dae_fun = Function('dae_'+name,sys_dae,["x","u","z","p","t"],["ode","alg"])
      self.add_function(sys_dae_fun)

    gridfun_options = {}
    if modern_casadi():
      gridfun_options["allow_free"] = True
    gridfun = Function("grid_"+name,
      parameters,
      [self.sample(self.t, grid='control')[1]],
      parameter_names,
      ['grid'], gridfun_options)
    if gridfun.has_free():
      gridfun = self.to_function("grid_"+name,
        parameters,
        [self.sample(self.t, grid='control')[1]],
        parameter_names,
        ['grid'])
    self.add_function(gridfun)

    costfun_options = {}
    if modern_casadi():
      costfun_options["allow_free"] = True

    costfun = Function("cost_"+name,
      [states]+[controls]+parameters,
      [self._method.opti.f],
      ['x','u'] + parameter_names,
      ['f'],costfun_options
    )
    if costfun.has_free():
      print("Cost function has stray dependencies")
    else:
      self.add_function(costfun)

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
        self.patch_codegen(casadi_codegen_file_name, self,qp_error_on_fail=qp_error_on_fail)
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


    pool_names = ["x_initial_guess","z_initial_guess","u_initial_guess","v_initial_guess","p","x_opt","z_opt","u_opt","v_opt","grid"]

    p_offsets = [0]
    for p in parameters:
      p_offsets.append(p_offsets[-1]+p.numel())

    v_offsets = [0]
    for v in variables:
      v_offsets.append(v_offsets[-1]+v.numel())

    p_nominal = self.initial_value(vvcat(parameters))
    x_nominal = self.initial_value(vec(states))
    z_nominal = self.initial_value(vec(algebraics))
    u_nominal = self.initial_value(vec(controls))
    v_nominal = self.initial_value(vvcat(variables))
    grid_nominal = self.initial_value(grid)

    if isinstance(p_nominal,float): p_nominal = np.array([p_nominal])
    if isinstance(x_nominal,float): x_nominal = np.array([x_nominal])
    if isinstance(z_nominal,float): z_nominal = np.array([z_nominal])
    if isinstance(u_nominal,float): u_nominal = np.array([u_nominal])
    if isinstance(v_nominal,float): v_nominal = np.array([v_nominal])
    if isinstance(grid_nominal,float): grid_nominal = np.array([grid_nominal])

    p_names = parameter_names
    x_names = [x.name() for x in self.states]
    z_names = [z.name() for z in self.algebraics]
    u_names = [u.name() for u in self.controls]
    v_names = variable_names
    grid_names = ["t"]


    hello_p_normal_name = None
    hello_p_normal = None
    hello_p_normal_nominal = None
    if self.parameters['']:
      hello_p_normal = self.parameters[''][-1]
      hello_p_normal_name = hello_p_normal.name()
      hello_p_normal_nominal = self.initial_value(self.value(hello_p_normal))
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


    v_part_offset = [0]
    v_part_unit = []
    for v_symbol,v_sampled in zip(variables_symbols,variables):
      v_part_unit.append(v_symbol.numel())
      v_part_offset.append(v_part_offset[-1]+v_sampled.numel())

    if i_x_current is None:
      raise Exception("You must define a parameter named 'x_current'")

    x_current_nominal = self.initial_value(parameters[i_x_current])

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
          {prefix}struct* m = initialize(printf, 0);
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
          int i, j, n_row, n_col, flag;
          double *u_scratch, *x_scratch;
          {prefix}struct* m = impact_initialize(printf, 0);
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


          printf("Start a solve.\\n");
          flag = impact_solve(m);
          if (flag) {{
            printf("Solve indicates a problem: return code %d.\\n", flag);
          }}
          printf("Solve finished.\\n");

          const impact_stats* stats = impact_get_stats(m);
          if (stats) {{
            printf("Number of outer iterations: %d\\n", stats->n_sqp_iter);
            printf("Stop criterion: %d\\n", stats->sqp_stop_crit);
            printf("Runtime [s]: %e\\n", stats->runtime);
          }} else {{
            printf("No stats available.\\n");
          }}

          impact_print_problem(m);

          impact_hotstart(m);

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
          free(u_scratch);
          free(x_scratch);

          impact_destroy(m);
        }}
      """
      )
    flags = OrderedDict([("ALL",0),("EVERYWHERE",-1),("FULL",0),("HREP",1),("VREP",2),("COLUMN_MAJOR",0),("ROW_MAJOR",4)])

    with open(h_file_name,"w") as out:
      for flag_name, flag_value in flags.items():
        out.write(f"#define IMPACT_{flag_name} {flag_value}\n")

      out.write(f"""

{write_struct(Struct(prefix+"stats", solver_stats_type.fields))}

struct {prefix}_struct;
typedef struct {prefix}_struct {prefix}struct;

typedef int (*formatter)(const char * s);

/**
  \\brief Initialize and instance of an impact library

  \\note Any call to \\c initialize must be followed up with a call to \\c destroy within the same process.

  \\param[in]  fp Function pointer to a printf-like function
              Common values: 0 (null pointer: no printing), printf (C's standard printf method defined in stdio)
  \\param[in]  build_dir Location of build directory. If not supplied (null pointer), uses a hard-coded export path.


  \\return Impact instance pointer, or a null pointer upon failure

*/
{prefix}struct* {prefix}initialize(formatter fp, const char* build_dir);
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

  \\parameter[in] Hotstart

  \\return 0 indicates success

*/
int {prefix}solve({prefix}struct* m);

/*
Prepare internal structure such that the next solve is hotstarted with the current solution
*/
void {prefix}hotstart({prefix}struct* m);

/*
Prepare internal structure such that no hotstart information is carried over
*/
void {prefix}coldstart({prefix}struct* m);

/*
Prepare internal structure such that the next solve uses the nominal trajectory from export
*/
void {prefix}freshstart({prefix}struct* m);

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
                               Any NaNs are ignored (i.e. the memory pool is untouched for those entries)
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
/**
  \\brief Get number of (scalarized) variables
*/
int {prefix}get_nv();

int {prefix}get_id_count({prefix}struct* m, const char* pool_name);
int {prefix}get_id_from_index({prefix}struct* m, const char* pool_name, int index, const char** id);
const {prefix}stats* {prefix}get_stats(const {prefix}struct* m);

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
    
    grid_part_offset = [0,1]
    grid_part_unit = [1]

    p_trajectory_length = [1 for p in self.parameters['']]+[self._method.N for p in self.parameters['control']]+[self._method.N+1 for p in self.parameters['control+']]
    x_trajectory_length = [self._method.N+1 for x in self.states]
    z_trajectory_length = [self._method.N+1 for x in self.algebraics]
    u_trajectory_length = [self._method.N for x in self.controls]
    x_current_trajectory_length = [1 for x in self.states]
    v_trajectory_length = [1 for v in self.variables['']]+[self._method.N for v in self.variables['control']]+[self._method.N+1 for v in self.variables['control+']]
    grid_trajectory_length = [self._method.N+1]

    p_part_stride = p_part_unit
    x_part_stride = [self.nx for x in self.states]
    z_part_stride = [self.nz for x in self.algebraics]
    u_part_stride = [self.nu for u in self.controls]
    v_part_stride = v_part_unit
    grid_part_stride = grid_part_unit

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
            {prefix}pool* v_initial_guess;
            {prefix}pool* p;


            {prefix}pool* x_opt;
            {prefix}pool* z_opt;
            {prefix}pool* u_opt;
            {prefix}pool* v_opt;
            {prefix}pool* grid;

            casadi_real* hotstart_in;
            casadi_real* hotstart_out;

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
      prim = ["x","z","u","p","v"]
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
        yield ("grid_names","char*")
        yield ("grid_nominal","casadi_real")
        yield ("grid_part_offset","casadi_int")
        yield ("grid_part_unit","casadi_int")
        yield ("grid_part_stride","casadi_int")
        yield ("grid_trajectory_length","casadi_int")

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


          {prefix}struct* {prefix}initialize(formatter fp, const char* build_dir) {{
            int flag;
            int i;
            {prefix}struct* m;
            char casadi_file[2048];
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
              if (build_dir==0) {{
                strcpy(casadi_file, "{escape(build_dir_abs)}");
              }} else {{
                strcpy(casadi_file, build_dir);
              }}
              strcat(casadi_file, "/{name}.casadi");
              flag = casadi_c_push_file(casadi_file);
              m->pop = 1;
              if (flag) {{
                m->fatal(m, "initialize", "Could not load file '%s'.\\n", casadi_file);
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
            m->p->data = malloc(sizeof(casadi_real)*{max(p_nominal.size,1)});
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
      """)

      for sname,d in [("x",{"nominal":x_nominal,"sym":self.states,"stride":self.nx}),
                     ("z",{"nominal":z_nominal,"sym":self.algebraics,"stride":self.nz}),
                     ("u",{"nominal":u_nominal,"sym":self.controls,"stride":self.nu}),
                     ("v",{"nominal":v_nominal,"sym":variables,"stride":-1})]:
        out.write(f"""
            m->{sname}_initial_guess = malloc(sizeof({prefix}pool));
            m->{sname}_initial_guess->names = {prefix}{sname}_names;
            m->{sname}_initial_guess->size = {d["nominal"].size};
            m->{sname}_initial_guess->data = malloc(sizeof(casadi_real)*{max(d["nominal"].size,1)});
            m->{sname}_initial_guess->n = {len(d["sym"])};
            m->{sname}_initial_guess->trajectory_length = {prefix}{sname}_trajectory_length;
            m->{sname}_initial_guess->stride = {d["stride"]};
            m->{sname}_initial_guess->part_offset = {prefix}{sname}_part_offset;
            m->{sname}_initial_guess->part_unit = {prefix}{sname}_part_unit;
            m->{sname}_initial_guess->part_stride = {prefix}{sname}_part_stride;

            m->{sname}_opt = malloc(sizeof({prefix}pool));
            m->{sname}_opt->names = {prefix}{sname}_names;
            m->{sname}_opt->size = {d["nominal"].size};
            m->{sname}_opt->data = malloc(sizeof(casadi_real)*{max(d["nominal"].size,1)});
            m->{sname}_opt->n = {len(d["sym"])};
            m->{sname}_opt->trajectory_length = {prefix}{sname}_trajectory_length;
            m->{sname}_opt->stride = {d["stride"]};
            m->{sname}_opt->part_offset = {prefix}{sname}_part_offset;
            m->{sname}_opt->part_unit = {prefix}{sname}_part_unit;
            m->{sname}_opt->part_stride = {prefix}{sname}_part_stride;
            """)

      out.write(f"""
            m->grid = malloc(sizeof({prefix}pool));
            m->grid->names = {prefix}grid_names;
            m->grid->size = {grid_nominal.size};
            m->grid->data = malloc(sizeof(casadi_real)*{max(grid_nominal.size,1)});
            m->grid->n = 1;
            m->grid->trajectory_length = {prefix}grid_trajectory_length;
            m->grid->stride = 1;
            m->grid->part_offset = {prefix}grid_part_offset;
            m->grid->part_unit = {prefix}grid_part_unit;
            m->grid->part_stride = {prefix}grid_part_stride;
          """)
        
      out.write(f"""
            m->hotstart_in = malloc(sizeof(casadi_real)*{max(hotstart_symbol.numel(),1)});
            m->hotstart_out = malloc(sizeof(casadi_real)*{max(hotstart_symbol.numel(),1)});

            memcpy(m->p->data, {prefix}p_nominal, {p_nominal.size}*sizeof(casadi_real));
            memcpy(m->x_opt->data, {prefix}x_nominal, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->z_opt->data, {prefix}z_nominal, {z_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_opt->data, {prefix}u_nominal, {u_nominal.size}*sizeof(casadi_real));
            memcpy(m->v_opt->data, {prefix}v_nominal, {v_nominal.size}*sizeof(casadi_real));
            memcpy(m->grid->data, {prefix}grid_nominal, {grid_nominal.size}*sizeof(casadi_real));
            for (i=0;i<{hotstart_symbol.numel()};++i) m->hotstart_out[i] = 0.0;
            {prefix}freshstart(m);
            return m;
          }}

          void {prefix}coldstart({prefix}struct* m) {{
            int i;
            for (i=0;i<{hotstart_symbol.numel()};++i) m->hotstart_in[i] = 0.0;
          }}

          void {prefix}freshstart({prefix}struct* m) {{
            int i;
            memcpy(m->x_initial_guess->data, {prefix}x_nominal, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->z_initial_guess->data, {prefix}z_nominal, {z_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_initial_guess->data, {prefix}u_nominal, {u_nominal.size}*sizeof(casadi_real));
            memcpy(m->v_initial_guess->data, {prefix}v_nominal, {v_nominal.size}*sizeof(casadi_real));
            for (i=0;i<{hotstart_symbol.numel()};++i) m->hotstart_in[i] = 0.0;
          }}

          void {prefix}hotstart({prefix}struct* m) {{
            int i;
            memcpy(m->x_initial_guess->data, m->x_opt->data, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->z_initial_guess->data, m->z_opt->data, {z_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_initial_guess->data, m->u_opt->data, {u_nominal.size}*sizeof(casadi_real));
            memcpy(m->v_initial_guess->data, m->v_opt->data, {v_nominal.size}*sizeof(casadi_real));
            memcpy(m->hotstart_in, m->hotstart_out, {hotstart_symbol.numel()}*sizeof(casadi_real));
          }}

          void {prefix}destroy({prefix}struct* m) {{
            if (m) {{
              /* Free memory (thread-safe) */
              {casadi_call("decref")};
              /* Release thread-local (not thread-safe) */
              {casadi_call("release","m->mem")};
              {"" if use_codegen else "if (m->pop) casadi_c_pop();"}
              /*free(m->arg);
              free(m->res);
              free(m->iw);
              free(m->w);
              free(m->p->data);
              free(m->p);
              free(m->x_current);
              free(m->x_initial_guess->data);
              free(m->x_initial_guess);
              free(m->z_initial_guess->data);
              free(m->z_initial_guess);
              free(m->u_initial_guess->data);
              free(m->u_initial_guess);
              free(m->v_initial_guess->data);
              free(m->v_initial_guess);
              free(m->hotstart_in);
              free(m->hotstart_out);
              free(m->x_opt->data);
              free(m->x_opt);
              free(m->z_opt->data);
              free(m->z_opt);
              free(m->u_opt->data);
              free(m->u_opt);
              free(m->v_opt->data);
              free(m->v_opt);
              free(m->grid->data);
              free(m->grid);*/
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
            }} else if (!strcmp(name,"v_initial_guess")) {{
              return  m->v_initial_guess;
            }} else if (!strcmp(name,"x_opt")) {{
              return m->x_opt;
            }} else if (!strcmp(name,"z_opt")) {{
              return m->z_opt;
            }} else if (!strcmp(name,"u_opt")) {{
              return m->u_opt;
            }} else if (!strcmp(name,"v_opt")) {{
              return m->v_opt;
            }} else if (!strcmp(name,"grid")) {{
              return m->grid;
            }} else {{
              m->fatal(m, "get_pool_by_name", "Pool with name '%s' not recognized. \
                                       Use one of: {pool_names}. \\n", name);
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
              if (p->stride>=0) {{
                *n_row = p->stride;
                *n_col = p->trajectory_length[0];
              }} else {{
                *n_row = p->size;
                *n_col = 1;
              }}
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
          int {prefix}get_nv() {{ return {self.nv}; }}

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
            m->arg_casadi[3] = m->v_initial_guess->data;
            for (i=0;i<{len(parameters)};i++) {{
              m->arg_casadi[4+i] = m->p->data + {prefix}p_offsets[i];
            }}
            m->arg_casadi[4+{len(parameters)}] = m->hotstart_in;
            m->res_casadi[0] = m->x_opt->data;
            m->res_casadi[1] = m->z_opt->data;
            m->res_casadi[2] = m->u_opt->data;
            m->res_casadi[3] = m->v_opt->data;
            m->res_casadi[4] = m->hotstart_out;
            m->res_casadi[5] = m->grid->data;
            return {casadi_call("eval","m->arg_casadi","m->res_casadi","m->iw_casadi","m->w_casadi","m->mem")};
          }}

          int {prefix}get_p_by_id({prefix}struct* m, const char* id, const casadi_real* value) {{
            return 0;
          }}

          void {prefix}get_u({prefix}struct* m, double* value) {{

          }}

          const {prefix}stats* {prefix}get_stats(const {prefix}struct* m) {{
            return {casadi_call("stats") if use_codegen else 0};
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

        {write_struct(solver_stats_type,target="matlab_c")}

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
            m = {prefix}initialize(mexPrintf, 0);
#else
            m = {prefix}initialize(0, 0);
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
            if (!ssSetNumInputPorts(S, n_p+{2+(self.nz>0)+(self.nv>0)}+1)) return;
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
            """)

      if self.nv>0:
        out.write(f"""
            {prefix}get_size(m, "v_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            ssSetInputPortRequiredContiguous(S, i, 1);
            i++;""")

      out.write(f"""
            // Port
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, 1, 1);
            ssSetInputPortRequiredContiguous(S, i, 1);
            i++;
        """)
      

      
      def ports():
        yield (f"""{prefix}get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, {'1' if short_output else 'n_col'});""",
            f"""{prefix}get(m, "x_opt", IMPACT_ALL, {'1' if short_output else 'IMPACT_EVERYWHERE'}, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);""")
        if self.nz>0:
            yield (f"""
          {prefix}get_size(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
          ssSetOutputPortMatrixDimensions(S, i, n_row, n_col);""",
            f"""{prefix}get(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);"""
            )
        yield (f"""
            {prefix}get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, {'1' if short_output else 'n_col'});
            """,
            f"""{prefix}get(m, "u_opt", IMPACT_ALL, {'0' if short_output else 'IMPACT_EVERYWHERE'}, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);""")
        
        if self.nv>0:
          yield (f"""
            {prefix}get_size(m, "v_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
          ssSetOutputPortMatrixDimensions(S, i, n_row, n_col);
          """,
          f"""{prefix}get(m, "v_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);"""
            )

        if ignore_errors:
          yield (f"""
              ssSetOutputPortMatrixDimensions(S, i, 1, 1);
              """,
              """ssGetOutputPortRealSignal(S, i++)[0] = ret;"""
              )
          
        yield (f"""
#if MATLAB_MEX_FILE
            if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {{
                DTypeId tid;
                ssRegisterTypeFromNamedObject(S, "solver_stats_bus", &tid)
                if (tid==INVALID_DTYPE_ID) {{
                  mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                                    "Could not find solver_stats_bus data_type.");
                }}
                ssSetOutputPortDataType(S, i, tid);
            }}
#endif  
            ssSetOutputPortWidth(S, i, 1);
            ssSetOutputPortBusMode(S, i, SL_BUS_MODE);
            ssSetBusOutputObjectName(S, i, (void *) "{solver_stats_type.name}");
            ssSetBusOutputAsStruct(S, i, 1);
            """,
            f"""{solver_stats_type.name}* stats = ({solver_stats_type.name}*) ssGetOutputPortRealSignal(S, i++);
          const {prefix}stats* s = {prefix}get_stats(m);"""
            )
        if not short_output:
          yield (f"""
            {prefix}get_size(m, "grid", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, i, n_row, n_col);
          """,
          f"""{prefix}get(m, "grid", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, i++), IMPACT_FULL);"""
          )
        
      out.write(f"""
            if (!ssSetNumOutputPorts(S, {len(list(ports()))})) return;
            i = 0;
            """)
      for init,_ in ports():
        out.write(f"""
            {init}
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
            """)
      if self.nv>0:
        out.write(f"""
            {prefix}set(m, "v_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetInputPortSignal(S,i++), IMPACT_FULL);""")

      out.write(f"""
            const real_T* hotstart_ptr = ssGetInputPortSignal(S,i++);
            real_T hotstart = *hotstart_ptr;

            if (hotstart==1.0) {{
              {prefix}hotstart(m);
            }} else if (hotstart==-1.0) {{
              {prefix}freshstart(m);
            }} else if (hotstart==0.0) {{
              {prefix}coldstart(m);
            }}

            int_T ret = {prefix}solve(m);
            {prefix}print_problem(m);
            if (ret && {int(not ignore_errors)}) {{
                static char msg[{200+len(s_function_name)}];
                sprintf(msg, "SFunction '{s_function_name}' failed to compute (error code %d) at t=%.6fs. "
                "Export with ('ignore_errors', true) if you want the simulation to continue anyway.", ret, ssGetT(S));
                #ifdef ssSetLocalErrorStatus
                  ssSetLocalErrorStatus(S, msg);
                #else
                  #ifdef ssSetErrorStatus
                    ssSetErrorStatus(S, msg);
                  #endif
                #endif
            }}

            i = 0;""")
      for _,update in ports():
        out.write(f"""
          {update}
        """)
          
      for f in solver_stats_type.fields:
        out.write(f"          stats->{f.name} = s ? s->{f.name} : 0;\n")

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
      fun.save(os.path.join(build_dir_abs, fun.name()+".casadi"))
      added_functions.append(fun2s_function(fun, dir=build_dir_abs,ignore_errors=ignore_errors,build_dir_abs=build_dir_abs))

    m_build_file_name = os.path.join(build_dir_abs,"build.m")
    with open(m_build_file_name,"w") as out:
      out.write("""
      try
        casadi.MX;
      catch
        error('This block depends on CasADi. Obtain it from install.casadi.org and make sure to have the casadi directory added to the matlab path.');
      end

      """)
      out.write(f"""
        s_function_file_name_base = '{s_function_file_name_base}';
        c_file_name_base = '{c_file_name_base}';
        casadi_codegen_file_name_base = '{casadi_codegen_file_name_base}';
        [build_dir,~,~] = fileparts(mfilename('fullpath'));
        flags={{['-I' build_dir] ['-L' build_dir] '-DCASADI_PRINTF=mexPrintf' ['-I' casadi.GlobalOptions.getCasadiIncludePath()] ['-L' casadi.GlobalOptions.getCasadiPath()]}};
        files = {{[build_dir filesep s_function_file_name_base], [build_dir filesep c_file_name_base]}};
        """)
      for e in artifacts:
        if ".c" in e.name:
          out.write(f"""           files = [files {{[build_dir filesep '{os.path.basename(e.name)}']}}];\n""")
      if use_codegen:

        ipopt = ''
        if modern_casadi():
          ipopt = "'-lipopt' '-lfatrop' '-lblasfeo'"
        out.write(f"""
          files=[files {{ [build_dir filesep casadi_codegen_file_name_base]}}];
          flags=[flags {{['-I' casadi.GlobalOptions.getCasadiIncludePath()] ['-I' casadi.GlobalOptions.getCasadiPath()] '-losqp' {ipopt}}}];
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
    if self.nv>0:
      port_labels_in.append('v_initial_guess')
    port_labels_in.append('hotstart')
    mask_name = f"IMPACT MPC\\n{name}"
    port_labels_out = []
    port_labels_out.append("x_opt @ k=1" if short_output else "x_opt")
    if self.nz>0:
      port_labels_out.append("z_opt")
    port_labels_out.append("u_opt @ k=0" if short_output else "u_opt")
    if self.nv>0:
      port_labels_out.append("v_opt")
    if ignore_errors:
      port_labels_out.append("status code (0=good)")
    port_in = []
    for p in parameters_symbols:
      port_in.append(f"NaN({p.shape[0]},{p.shape[1]})")

    port_in.append(f"NaN({self.nx},{self._method.N+1})")
    if self.nz>0:
      port_in.append(f"NaN({len(self.algebraics)},{self._method.N+1})")
    port_in.append(f"NaN({self.nu},{self._method.N})")
    if self.nv>0:
      port_in.append(f"NaN({v_nominal.size},1)")
    port_in.append(f"0")


    port_labels_out.append("solver stats")
    dependencies = f"""
    try
      casadi.load_nlpsol('ipopt')
      casadi.load_nlpsol('fatrop')
      casadi.load_conic('osqp')
    catch
      error('This block depends on CasADi >= 3.6.7 - Obtain it from install.casadi.org and make sure to have the casadi directory added to the matlab path.');
    end

    """

    if not short_output:
      port_labels_out.append("grid")
    mask = Mask(port_labels_in=port_labels_in, port_labels_out=port_labels_out,port_in=port_in,block_name=name,name=mask_name,s_function=s_function_name,dependencies=[name+"_codegen", name],init_code=dependencies+write_bus(solver_stats_type))

    diag.add(mask)
    for fname,fun in zip(added_functions,self._added_functions):
      mask = Mask(port_labels_in=fun.name_in(), port_labels_out=fun.name_out(), block_name=fun.name(), name=fun.name(), s_function=fname[:-2])
      diag.add(mask)
    diag.write()


    make_file_name = os.path.join(build_dir_abs,"Makefile")

    source_files = [casadi_codegen_file_name]
    for e in artifacts:
      if ".c" in e.name:
        source_files.append(os.path.join(build_dir_abs, e.name))
      
    with open(make_file_name,"w") as out:

      flags = ["-pedantic","-Wall","-Wextra","-Wno-unknown-pragmas","-Wno-long-long","-Wno-unused-parameter","-Wno-unused-const-variable","-Wno-sign-compare","-Wno-unused-but-set-variable","-Wno-unused-variable","-Wno-endif-labels"]
      deps = ["-I"+build_dir_abs,"-L"+build_dir_abs,"-Wl,-rpath="+build_dir_abs,"-L"+GlobalOptions.getCasadiPath(),"-I"+GlobalOptions.getCasadiIncludePath(),"-Wl,-rpath="+GlobalOptions.getCasadiPath()]
      if use_codegen:
        lib_codegen_file_name = os.path.join(build_dir_abs,"lib" + name + "_codegen.so")
        lib_codegen_compile_commands = ["gcc"]+c_flags+["-fPIC","-shared"]+source_files+ ["-lm","-o"+lib_codegen_file_name]+deps
        deps += ["-l"+name+"_codegen","-losqp"]

        if modern_casadi():
          deps += ["-lipopt","-lfatrop","-lblasfeo"]
        fatrop_driver = os.path.join(os.path.abspath(src_dir),"foobar","lib","libfatrop_driver.so")
        if os.path.exists(fatrop_driver):
          shutil.copy(fatrop_driver, build_dir_abs)
          shutil.copy(os.path.join(os.path.abspath(src_dir),"foobar","lib","libfatrop.so"), build_dir_abs)
          deps += ["-lfatrop_driver"]

      else:
        deps += ["-lcasadi"]
      lib_compile_commands = ["gcc"]+c_flags+["-fPIC","-shared",c_file_name,"-o"+lib_file_name]+deps+flags

      hello_compile_commands = ["gcc"]+c_flags+[hello_c_file_name,"-I"+build_dir_abs,"-l"+lib_name,"-o",hello_file_name]+deps+flags

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
        out.write(os.path.basename(lib_codegen_file_name) + ": " + " ".join([os.path.basename(e) for e in source_files]) + "\n")
        out.write("\t"+" ".join(lib_codegen_compile_commands)+"\n\n")

      out.write(os.path.basename(lib_file_name) + ": " + os.path.basename(c_file_name) + "\n")
      out.write("\t"+" ".join(lib_compile_commands)+"\n\n")

      out.write(os.path.basename(hello_file_name) + ": " + os.path.basename(hello_c_file_name) + "\n")
      out.write("\t"+" ".join(hello_compile_commands)+"\n\n")

    cmake_file_name = os.path.join(build_dir_abs,"CMakeLists.txt")
    with open(cmake_file_name,"w") as out:
      out.write(f"""
      project({name})

      set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

      cmake_minimum_required(VERSION 3.0)

      message("If you get a complaint about missing libcasadi.lib, please copy the casadi.lib file to libcasadi.lib")
      set(CMAKE_PREFIX_PATH "{escape(casadi.GlobalOptions.getCasadiPath())}" ${{CMAKE_PREFIX_PATH}})
      find_package(casadi REQUIRED)

      add_library({name} SHARED {name}.c)
      target_link_libraries({name} casadi)

      install(TARGETS {name} RUNTIME DESTINATION . LIBRARY DESTINATION .)

      """)
    cmake_file_name = os.path.join(build_dir_abs,"build.bat")
    with open(cmake_file_name,"w") as out:
      out.write(f"""
      echo "Should be ran in 'x64 Native Tools Command Prompt for VS'"
      cmake -G "Visual Studio 16 2019" -A x64 -B build
      cmake --build build --config Release
      cmake --install build --prefix .
      """)
    cmake_file_name = os.path.join(build_dir_abs,"build.sh")
    with open(cmake_file_name,"w") as out:
      out.write(f"""
      cmake -DCMAKE_INSTALL_PREFIX=. -B build
      cmake --build build --config Release
      cmake --install build --prefix .
      """)
    print("success")

  def save(self,name):
      self._untranscribe()
      import pickle
      with rockit_pickle_context():
          pickle.dump(self,open(name,"wb"))

  @staticmethod
  def load(name):
      import pickle
      with rockit_unpickle_context():
          return pickle.load(open(name,"rb"))
