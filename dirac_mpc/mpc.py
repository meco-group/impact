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

class MPC(Ocp):
  def __init__(self, **kwargs):
    Ocp.__init__(self, **kwargs)
    self._expr = Model()
    self.basename = os.path.dirname(__file__)

  @property
  def expr(self):
      return self._expr

  def parameter(self,name,*args,**kwargs):
    p = MX.sym(name,*args)
    self.register_parameter(p,**kwargs)
    self.expr._register('p', {name: p})
    return p

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

    with open(name,'w') as fout:
      for line in lines:
        # Bugfix https://gitlab.kuleuven.be/meco/projects/sbo_dirac/dirac_mpc/-/issues/11
        if "return fmax(x, y);" not in line and "CASADI_PREFIX" not in line:
          line = re.sub(r"\bfmax\b","casadi_fmax", line)
        fout.write(line)



  def export(self,name,src_dir=".",use_codegen=None):
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

    casadi_codegen_file_name = os.path.join(build_dir_abs,name+"_codegen.c")
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

    def strlist(a):
      return ",".join('"'+e+'"' if isinstance(e,str) else str(e) for e in a)


    pool_names = ["x_initial_guess","z_initial_guess","u_initial_guess","p","x_opt","z_opt","u_opt"]

    p_offsets = [0]
    for p in parameters:
      p_offsets.append(p_offsets[-1]+p.numel())

    p_nominal = self._method.opti.value(vvcat(parameters),self._method.opti.initial())
    x_nominal = self._method.opti.value(vec(states),self._method.opti.initial())
    z_nominal = self._method.opti.value(vec(algebraics),self._method.opti.initial())
    u_nominal = self._method.opti.value(vec(controls),self._method.opti.initial())



    np = vvcat(parameters).numel()



    p_names = ['"'+p.name()+'"' for p in self.parameters['']+self.parameters['control']]
    x_names = ['"'+x.name()+'"' for x in self.states]
    z_names = ['"'+z.name()+'"' for z in self.algebraics]
    u_names = ['"'+u.name()+'"' for u in self.controls]

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

    c_file_name = os.path.join(build_dir_abs,name+".c")
    h_file_name = os.path.join(build_dir_abs,name+".h")
    hello_file_name = os.path.join(build_dir_abs,"hello_world_"+name)
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

          double x0[{x_current_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_current_nominal)} }};

          impact_set(m, "x_current", IMPACT_ALL, IMPACT_EVERYWHERE, x0, IMPACT_FULL);

          double num = {"%0.16f" % x_current_nominal[0]};

          impact_set(m, "x_current", "{self.states[0].name()}", IMPACT_EVERYWHERE, &num, IMPACT_FULL);

          impact_print_problem(m);

          impact_solve(m);


          // Allocate scratch space for state and control trajectories
          impact_get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
          printf("u_opt dims: %d - %d\\n", n_row, n_col);
          //u_scratch = malloc(sizeof(double)*n_row*n_col);
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
 /* @{{ **
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


    p_part_stride = p_part_unit
    x_part_stride = [self.nx for x in self.states]
    z_part_stride = [self.nz for x in self.algebraics]
    u_part_stride = [self.nu for u in self.controls]


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

          // For printing
          #include <stdio.h>
          #include <stdarg.h>

          static const casadi_int {prefix}p_offsets[{len(parameters)+1}] = {{ {strlist(p_offsets)} }};
          static const casadi_int {prefix}x_part_offset[{len(x_part_offset)}] = {{ {strlist(x_part_offset)} }};
          static const casadi_int {prefix}z_part_offset[{len(z_part_offset)}] = {{ {strlist(z_part_offset)} }};
          static const casadi_int {prefix}u_part_offset[{len(u_part_offset)}] = {{ {strlist(u_part_offset)} }};
          static const casadi_int {prefix}p_part_offset[{len(p_part_offset)}] = {{ {strlist(p_part_offset)} }};
          static const casadi_int {prefix}x_part_unit[{len(x_part_unit)}] = {{ {strlist(x_part_unit)} }};
          static const casadi_int {prefix}z_part_unit[{len(z_part_unit)}] = {{ {strlist(z_part_unit)} }};
          static const casadi_int {prefix}u_part_unit[{len(u_part_unit)}] = {{ {strlist(u_part_unit)} }};
          static const casadi_int {prefix}p_part_unit[{len(p_part_unit)}] = {{ {strlist(p_part_unit)} }};

          static const casadi_int {prefix}x_part_stride[{len(x_part_unit)}] = {{ {strlist(x_part_stride)} }};
          static const casadi_int {prefix}z_part_stride[{len(z_part_unit)}] = {{ {strlist(z_part_stride)} }};
          static const casadi_int {prefix}u_part_stride[{len(u_part_unit)}] = {{ {strlist(u_part_stride)} }};
          static const casadi_int {prefix}p_part_stride[{len(p_part_unit)}] = {{ {strlist(p_part_stride)} }};

          static const char* {prefix}p_names[{len(parameters)}] = {{ {",".join(p_names)} }};
          static const char* {prefix}x_names[{len(self.states)}] = {{ {",".join(x_names)} }};
          static const char* {prefix}z_names[{len(self.algebraics)}] = {{ {",".join(z_names)} }};
          static const char* {prefix}u_names[{len(self.controls)}] = {{ {",".join(u_names)} }};

          static const casadi_int {prefix}p_trajectory_length[{len(parameters)}] = {{ {strlist(p_trajectory_length)} }};
          static const casadi_int {prefix}x_trajectory_length[{len(self.states)}] = {{ {",".join(str(self._method.N+1) for x in self.states)} }};
          static const casadi_int {prefix}z_trajectory_length[{len(self.algebraics)}] = {{ {",".join(str(self._method.N+1) for x in self.algebraics)} }};
          static const casadi_int {prefix}u_trajectory_length[{len(self.controls)}] = {{ {",".join(str(self._method.N) for u in self.controls)} }};
          static const casadi_int {prefix}x_current_trajectory_length[{len(self.states)}] = {{ {",".join("1" for x in self.states)} }};

          static const casadi_real {prefix}p_nominal[{p_nominal.size}] = {{ {",".join("%0.16f" % e for e in p_nominal)} }};
          static const casadi_real {prefix}u_nominal[{u_nominal.size}] = {{ {",".join("%0.16f" % e for e in u_nominal)} }};
          static const casadi_real {prefix}x_nominal[{x_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_nominal)} }};
          static const casadi_real {prefix}z_nominal[{z_nominal.size}] = {{ {",".join("%0.16f" % e for e in z_nominal)} }};
          static const casadi_real {prefix}x_current_nominal[{x_current_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_current_nominal)} }};

          static const char* {prefix}pool_names[{len(pool_names)}] = {{ {strlist(pool_names)} }};


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
              // Release thread-local (not thread-safe)
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
            // We might want to be adding other working memory here
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

            // Determine index
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

            // Determine index
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

    flags = ["-pedantic","-Wextra","-Wno-unknown-pragmas","-Wno-long-long","-Wno-unused-parameter","-Wno-unused-const-variable","-Wno-sign-compare","-Wno-unused-but-set-variable","-Wno-unused-variable","-Wno-endif-labels","-Wno-comment"]
    deps = ["-L"+build_dir_abs,"-Wl,-rpath="+build_dir_abs]
    if os.name!="nt":
      import subprocess
      if use_codegen:
        lib_codegen_file_name = os.path.join(build_dir_abs,"lib" + name + "_codegen.so")
        p = subprocess.run(["gcc","-g","-fPIC","-shared",casadi_codegen_file_name,"-lm","-o"+lib_codegen_file_name]+deps, capture_output=True, text=True)
        print(" ".join(p.args))
        if p.returncode!=0:
          raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))
        deps += ["-l"+name+"_codegen"]
      else:
        deps += ["-lcasadi","-L"+GlobalOptions.getCasadiPath(),"-I"+GlobalOptions.getCasadiIncludePath(),"-Wl,-rpath="+GlobalOptions.getCasadiPath()]

      p = subprocess.run(["gcc","-g","-fPIC","-shared",c_file_name,"-o"+lib_file_name]+deps+flags, capture_output=True, text=True)
      print(" ".join(p.args))
      if p.returncode!=0:
        raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))
      # #breaks matlab
      #p = subprocess.run(["gcc","-g",hello_c_file_name,"-I"+build_dir_abs,"-l"+lib_name,"-o",hello_file_name]+deps+flags, capture_output=True, text=True)
      #   if p.returncode!=0:
      ##  raise Exception("Failed to compile:\n{args}\n{stdout}\n{stderr}".format(args=" ".join(p.args),stderr=p.stderr,stdout=p.stdout))

    s_function_name = name+"_s_function_level2"

    s_function_file_name = os.path.join(build_dir_abs,s_function_name+".c")
      
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
            if (!ssSetNumInputPorts(S, n_p+3)) return;
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
            }}

            {prefix}get_size(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            i++;

            {prefix}get_size(m, "z_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            i++;

            {prefix}get_size(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetInputPortDirectFeedThrough(S, i, 1);
            ssSetInputPortMatrixDimensions(S, i, n_row, n_col);
            i++;

            if (!ssSetNumOutputPorts(S, 3)) return;

            {prefix}get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, 0, n_row, n_col);

            {prefix}get_size(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, 1, n_row, n_col);

            {prefix}get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
            ssSetOutputPortMatrixDimensions(S, 2, n_row, n_col);

            ssSetNumSampleTimes(S, 1);
            
            ssSetNumNonsampledZCs(S, 0);

            /* specify the sim state compliance to be same as a built-in block */
            ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

            /* Make sure mdlTerminate is called on error */
            ssSetOptions(S,
                        SS_OPTION_WORKS_WITH_CODE_REUSE |
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
              {prefix}set(m, "p", id, IMPACT_EVERYWHERE, *ssGetInputPortRealSignalPtrs(S,i), IMPACT_FULL);
            }}
            {prefix}set(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, *ssGetInputPortRealSignalPtrs(S,i), IMPACT_FULL);
            i++;
            {prefix}set(m, "z_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, *ssGetInputPortRealSignalPtrs(S,i), IMPACT_FULL);
            i++;
            {prefix}set(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, *ssGetInputPortRealSignalPtrs(S,i), IMPACT_FULL);
            i++;

            {prefix}solve(m);
            {prefix}print_problem(m);

            {prefix}get(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, 0), IMPACT_FULL);
            {prefix}get(m, "z_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, 1), IMPACT_FULL);
            {prefix}get(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, ssGetOutputPortRealSignal(S, 2), IMPACT_FULL);

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

    m_build_file_name = os.path.join(build_dir_abs,"build.m")
    with open(m_build_file_name,"w") as out:
      if use_codegen:
        out.write(f"""
          if ispc
            mex(['-I{build_dir_abs}'],['-I' casadi.GlobalOptions.getCasadiIncludePath()],['-L' casadi.GlobalOptions.getCasadiPath()],'-losqp',['-L{build_dir_abs}'],'{s_function_file_name}', '{c_file_name}', '{casadi_codegen_file_name}')
          else
            mex('-g',['-I{build_dir_abs}'],['-L{build_dir_abs}'],'-l{name}','LDFLAGS="\$LDFLAGS -Wl,-rpath,{build_dir_abs}"', '{s_function_file_name}')
          end
        """)
      else:
        out.write(f"""
          if ispc
            mex(['-I{build_dir_abs}'],['-I{GlobalOptions.getCasadiIncludePath()}'],['-L{build_dir_abs}'],['-L{GlobalOptions.getCasadiPath()}'],'-lcasadi', '{s_function_file_name}', '{c_file_name}')
          else
            mex('-g',['-I{build_dir_abs}'],['-I{GlobalOptions.getCasadiIncludePath()}'],['-L{build_dir_abs}'],['-L{GlobalOptions.getCasadiPath()}'],'-lcasadi','-l{name}','LDFLAGS="\$LDFLAGS -Wl,-rpath,{build_dir_abs}"', '{s_function_file_name}')
          end
        """)

    shutil.copy(os.path.join(self.basename,"templates","python","impact.py"), os.path.join(build_dir_abs,"impact.py"))
    py_file_name = os.path.join(build_dir_abs,"hello_world_" + name+".py")

      
    with open(py_file_name,"w") as out:
      out.write(f"""

import pylab as plt
import numpy as np

from impact import Impact

impact = Impact("cart_pendulum",src_dir="..")

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
    with ZipFile(template) as zipfile:
      zipfile.extractall(path=simulink_library_dirname)
    blockdiagram_filename = os.path.join(simulink_library_dirname,"simulink","blockdiagram.xml")
    with open(blockdiagram_filename,'r') as blockdiagram_file:
      tree = etree.parse(blockdiagram_file)

    system = tree.find('.//System')
    for e in system.findall('Block'):
      system.remove(e)
    for e in system.findall('Line'):
      system.remove(e)
    highwater = system.find('P[@Name="SIDHighWatermark"]')
    highwater.text = str(len(parameters))

    stride_height  = 40
    padding_height = 10
    unit_height  = 40

    margin_left = 100
    margin_top = 100

    constant_width = 30
    constant_height = 30

    line_width = 100
    sfun_width = 200

    constant_height_offset = 5


    for i,p in enumerate(parameters):
      id = i+2
      zorder = 7
      value = f"NaN({p.shape[0]},{p.shape[1]})"
      block = etree.fromstring(f"""
    <Block BlockType="Constant" Name="Constant" SID="{id}">
      <P Name="Position">[{margin_left}, {margin_top+i*stride_height+constant_height_offset}, {margin_left+constant_width}, {margin_top+constant_height+i*stride_height+constant_height_offset}]</P>
      <P Name="ZOrder">{zorder}</P>
      <P Name="Value">{value}</P>
      <P Name="VectorParams1D">off</P>
    </Block>
    """)
      system.append(block)
    sfun_id = i+3
    zorder = 8

    mask_commands = []
    for i,p in enumerate(parameters_symbols):
      mask_commands.append(f"port_label('input',{i+1},'{p.name()}');")
    mask_commands.append(f"port_label('input',{len(parameters)+1},'x_initial_guess');")
    mask_commands.append(f"port_label('input',{len(parameters)+2},'z_initial_guess');")
    mask_commands.append(f"port_label('input',{len(parameters)+3},'u_initial_guess');")
    mask_commands.append(f"disp('IMPACT MPC\\n{name}');")
    mask_commands.append(f"port_label('output',1,'x_opt');")
    mask_commands.append(f"port_label('output',2,'z_opt');")
    mask_commands.append(f"port_label('output',3,'u_opt');")
    mask_commands = "\n".join(mask_commands)
    block = etree.fromstring(f"""
    <Block BlockType="S-Function" Name="S-Function1" SID="{sfun_id}">
      <P Name="Ports">[{len(parameters)}, 2]</P>
      <P Name="Position">[{margin_left+constant_width+line_width}, {margin_top}, {margin_left+constant_width+line_width+sfun_width}, {margin_top+len(parameters)*stride_height}]</P>
      <P Name="ZOrder">{zorder}</P>
      <P Name="FunctionName">{s_function_name}</P>
      <P Name="SFunctionDeploymentMode">off</P>
      <P Name="EnableBusSupport">off</P>
      <P Name="SFcnIsStateOwnerBlock">off</P>
      <Object PropName="MaskObject" ObjectID="7" ClassName="Simulink.Mask">
        <P Name="Display" Class="char">{mask_commands}</P>
      </Object>
    </Block>
    """)
    system.append(block)

    for i,p in enumerate(parameters):
      id = i+2
      zorder = 7
      value = f"NaN({p.shape[0]},{p.shape[1]})"
      block = etree.fromstring(f"""
      <Line>
        <P Name="ZOrder">1</P>
        <P Name="Src">{id}#out:1</P>
        <P Name="Dst">{sfun_id}#in:{i+1}</P>
      </Line>
      """)
      system.append(block)


    tree.write(blockdiagram_filename)
    shutil.make_archive(simulink_library_filename,'zip',simulink_library_dirname)
    shutil.move(simulink_library_filename+".zip",simulink_library_filename)


    print("success")
