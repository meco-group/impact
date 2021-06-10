from rockit import Ocp
from casadi import Function, MX, vcat, vvcat, veccat, GlobalOptions, vec
import casadi
import yaml
import os


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
  def __init__(self, definition,prefix=""):
    if isinstance(definition,MX):
      self._symbols = [definition]
      self._concat = definition
      self._numel = definition
    else:
      symbols = []
      for d in definition:
        size = d["size"] if "size" in d else 1
        name = d["name"]
        symbols.append(MX.sym(prefix+name, size))
      self._symbols = symbols
      self._concat = vcat(symbols)
      self._numel = self._concat.numel()

  def __MX__(self):
    return self._concat


class Model(DotDict):
  def __init__(self, prefix=""):
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

    
    m = Model(name+".")

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
    locals = dict(m.__dict__)
    for k,v in m.__dict__.items():
      if k.startswith(name+"."):
        locals[k[len(name)+1:]] = v
    if "constants" in model_meta:
      if "inline" in model_meta["constants"]:
        for k,v in model_meta["constants"]["inline"].items():
          try:
            constants[k] = eval(v,casadi.__dict__,locals)
          except:
            constants[k] = v
    m._register("c", constants)

    # Define Outputs
    outputs = {}
    definitions = casadi.__dict__
    locals = dict(m.__dict__)
    for k,v in m.__dict__.items():
      if k.startswith(name+"."):
        locals[k[len(name)+1:]] = v
    if "outputs" in model_meta:
      if "inline" in model_meta["outputs"]:
        for k,v in model_meta["outputs"]["inline"].items():
          outputs[k] = eval(v,casadi.__dict__,locals)
    m._register("y", outputs)


    self.expr._update({name: m})
    return m


  def export(self,name,dir="."):
    build_dir_rel = name+"_build_dir"
    build_dir_abs = os.path.join(dir,build_dir_rel)
    os.makedirs(build_dir_abs,exist_ok=True)

    [_,states] = self.sample(self.x,grid='control')
    [_,controls] = self.sample(self.u,grid='control-')

    parameters_symbols = self.parameters['']+self.parameters['control']
    parameters = []
    for p in self.parameters['']:
      parameters.append(self.value(p))
    for p in self.parameters['control']:
      parameters.append(self.sample(p,grid='control-')[1])
    casadi_fun_name = 'ocpfun'
    ocpfun = self.to_function(casadi_fun_name,[states,controls]+parameters,[states,controls])

    casadi_file_name = os.path.join(build_dir_abs,name+".casadi")
    ocpfun.save(casadi_file_name)
    prefix = ""

    c_file_name = os.path.join(build_dir_abs,name+".c")
    h_file_name = os.path.join(build_dir_abs,name+".h")
    hello_file_name = os.path.join(build_dir_abs,"hello_world_"+name)
    hello_c_file_name = hello_file_name + ".c"
    with open(hello_c_file_name,"w") as out:
      out.write(f"""
        #include <{name}.h>
        #include <stdio.h>

        int main() {{
          MPCstruct* m = initialize(printf);
          if (!m) {{
            printf("Failed to initialize\\n");
            return 1;
          }}

          double x[4] = {{1,2,3,4}};
          //set(m, "x_current", x);
          set_repeat(m->x_initial_guess, x);

          get(m, "x_current", x);

          printf("x_current %f %f %f %f\\n", x[0], x[1], x[2], x[3]);

          double t = 42;
          set_by_id(m, "x_current", "scara.jointR_vel", &t );
          
          get(m, "x_current", x);

          printf("x_current %f %f %f %f\\n", x[0], x[1], x[2], x[3]);


          print_problem(m);


          //solve(m);



          destroy(m);
        }}
      """
      )

    with open(h_file_name,"w") as out:
      out.write(f"""
          #define casadi_real double
          #define casadi_int long long int

          typedef struct MPC_pool {{
            casadi_int n;
            casadi_int size;            
            const char** names; // length n
            const casadi_int* trajectory_length; // length n
            const casadi_int* part_offset; // length n
            const casadi_int* part_unit; // length n a non-full may span multiple parts
            const casadi_int* part_stride; // length n a non-full may span multiple parts
            casadi_real* data;
            casadi_int stride;
          }} MPCpool;

          struct MPCstruct;
          typedef struct MPC_struct MPCstruct;

          typedef int (*formatter)(const char * s);
          typedef void (*fatal_fp)(MPCstruct* m, const char * loc, const char * fmt, ...);
          typedef void (*info_fp)(MPCstruct* m, const char * fmt, ...);

          typedef struct MPC_struct {{
            int id;
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

            MPCpool* x_current;

            MPCpool* u_initial_guess;
            MPCpool* x_initial_guess;
            MPCpool* p;

            MPCpool* u_opt;
            MPCpool* x_opt;

            int mem;

            formatter fp;
            fatal_fp fatal;
            info_fp info;
          }} MPCstruct;

          MPCstruct* {prefix}initialize();
          void {prefix}destroy(MPCstruct* m);

          int {prefix}solve(MPCstruct* m);

          /*
          *   
          */
          int {prefix}set_by_id(MPCstruct* m, const char* var_name, const char* id, const casadi_real* value);
          int {prefix}set(MPCstruct* m, const char* var_name, const casadi_real* value);
          int {prefix}get(MPCstruct* m, const char* var_name, casadi_real* value);

          int {prefix}set_repeat(const MPCpool* p, const casadi_real* value);
          int {prefix}set_slice(const MPCpool* p, casadi_int i, const casadi_real* value);



          int {prefix}print_problem(MPCstruct* m);

      """
      )

    p_offsets = [0]
    for p in parameters:
      p_offsets.append(p_offsets[-1]+p.numel())

    p_nominal = self._method.opti.value(vvcat(parameters),self._method.opti.initial())
    x_nominal = self._method.opti.value(vec(states),self._method.opti.initial())
    u_nominal = self._method.opti.value(vec(controls),self._method.opti.initial())
    x_current_nominal = self._method.opti.value(states[:,0],self._method.opti.initial())

    np = vvcat(parameters).numel()


    """
    
    set_trajectory_by_id("joint1", m->x, value)
    set_repeat_by_id("joint1", m->x, value)
    set_slice_by_id("joint1", m->x, i, value)

    set_trajectory(m->x, value)
    set_repeat(m->x, value)
    set() # like repeat but trajectory_length = 1
    set_slice(m->x, i, value)

    """
    p_names = ['"'+p.name()+'"' for p in self.parameters['']+self.parameters['control']]
    x_names = ['"'+x.name()+'"' for x in self.states]
    u_names = ['"'+u.name()+'"' for u in self.controls]

    p_part_offset = [0]
    p_part_unit = []
    for p_symbol,p_sampled in zip(parameters_symbols,parameters):
      p_part_unit.append(p_symbol.numel())
      p_part_offset.append(p_part_offset[-1]+p_sampled.numel())

    x_part_offset = [0]
    x_part_unit = []
    for x in self.states:
      x_part_unit.append(x.numel())
      x_part_offset.append(x_part_offset[-1]+x.numel())

    u_part_offset = [0]
    u_part_unit = []
    for u in self.controls:
      u_part_unit.append(u.numel())
      u_part_offset.append(u_part_offset[-1]+u.numel())

    p_trajectory_length = [1 for p in self.parameters['']]+[self._method.N for p in self.parameters['control']]

    def strlist(a):
      return ",".join(str(e) for e in a)

    p_part_stride = p_part_unit
    u_part_stride = [self.nu for u in self.controls]
    x_part_stride = [self.nx for x in self.states]

    with open(c_file_name,"w") as out:
      out.write(f"""
          #include <stdlib.h>
          #include <string.h>          
          #include <casadi/casadi_c.h>
          #include "{name}.h"

          // For printing
          #include <stdio.h>
          #include <stdarg.h>

          static const casadi_int {prefix}p_offsets[{len(parameters)+1}] = {{ {strlist(p_offsets)} }};
          static const casadi_int {prefix}x_part_offset[{len(x_part_offset)}] = {{ {strlist(x_part_offset)} }};
          static const casadi_int {prefix}u_part_offset[{len(u_part_offset)}] = {{ {strlist(u_part_offset)} }};
          static const casadi_int {prefix}p_part_offset[{len(p_part_offset)}] = {{ {strlist(p_part_offset)} }};
          static const casadi_int {prefix}x_part_unit[{len(x_part_unit)}] = {{ {strlist(x_part_unit)} }};
          static const casadi_int {prefix}u_part_unit[{len(u_part_unit)}] = {{ {strlist(u_part_unit)} }};
          static const casadi_int {prefix}p_part_unit[{len(p_part_unit)}] = {{ {strlist(p_part_unit)} }};

          static const casadi_int {prefix}x_part_stride[{len(x_part_unit)}] = {{ {strlist(x_part_stride)} }};
          static const casadi_int {prefix}u_part_stride[{len(u_part_unit)}] = {{ {strlist(u_part_stride)} }};
          static const casadi_int {prefix}p_part_stride[{len(p_part_unit)}] = {{ {strlist(p_part_stride)} }};

          static const char* {prefix}p_names[{len(parameters)}] = {{ {",".join(p_names)} }};
          static const char* {prefix}x_names[{len(self.states)}] = {{ {",".join(x_names)} }};
          static const char* {prefix}u_names[{len(self.controls)}] = {{ {",".join(u_names)} }};

          static const casadi_int {prefix}p_trajectory_length[{len(parameters)}] = {{ {strlist(p_trajectory_length)} }};
          static const casadi_int {prefix}x_trajectory_length[{len(self.states)}] = {{ {",".join(str(self._method.N+1) for x in self.states)} }};
          static const casadi_int {prefix}u_trajectory_length[{len(self.controls)}] = {{ {",".join(str(self._method.N) for u in self.controls)} }};
          static const casadi_int {prefix}x_current_trajectory_length[{len(self.states)}] = {{ {",".join("1" for x in self.states)} }};

          static const casadi_real {prefix}p_nominal[{p_nominal.size}] = {{ {",".join("%0.16f" % e for e in p_nominal)} }};
          static const casadi_real {prefix}u_nominal[{u_nominal.size}] = {{ {",".join("%0.16f" % e for e in u_nominal)} }};
          static const casadi_real {prefix}x_nominal[{x_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_nominal)} }};
          static const casadi_real {prefix}x_current_nominal[{x_current_nominal.size}] = {{ {",".join("%0.16f" % e for e in x_current_nominal)} }};

          static const char* {prefix}pool_names[4] = {{"x_current","x_initial_guess","u_initial_guess","p"}};
          void {prefix}set_work(MPCstruct* m, const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w);
          void {prefix}work(MPCstruct* m, casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);



          void {prefix}fatal(MPCstruct* m, const char* loc, const char* fmt, ...) {{
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
          void {prefix}info(MPCstruct* m, const char* fmt, ...) {{
            va_list args;
            char buffer[1024];
            if (m->fp) {{
              va_start(args, fmt);
              vsprintf(buffer, fmt, args);
              m->fp(buffer);
              va_end(args);
            }}
          }}


          MPCstruct* {prefix}initialize(formatter fp) {{
            int flag;
            MPCstruct* m;
            m = malloc(sizeof(MPCstruct));
            m->fp = fp;
            m->fatal = {prefix}fatal;
            m->info = {prefix}info;
            flag = casadi_c_push_file("{casadi_file_name}");
            if (flag) {{
              m->fatal(m, "initialize", "Could not load file '{casadi_file_name}'.\\n");
              return 0;
            }}

            m->id = casadi_c_id("{casadi_fun_name}");
            if (m->id<0) {{
              m->fatal(m, "initialize", "Could locate function with name '{casadi_fun_name}'.\\n");
              {prefix}destroy(m);
              return 0;
            }}

            // Allocate memory (thread-safe)
            casadi_c_incref_id(m->id);
            // Checkout thread-local memory (not thread-safe)
            m->mem = casadi_c_checkout_id(m->id);

            {prefix}work(m, &m->sz_arg, &m->sz_res, &m->sz_iw, &m->sz_w);
            {prefix}set_work(m,
              malloc(sizeof(const casadi_real*)*m->sz_arg),
              malloc(sizeof(casadi_real*)*m->sz_res),
              malloc(sizeof(casadi_int)*m->sz_iw),
              malloc(sizeof(casadi_real)*m->sz_w));

            const casadi_real** arg = m->arg;
            m->arg_casadi = arg;

            casadi_real** res = m->res;
            m->res_casadi = res;

            casadi_int* iw = m->iw;
            m->iw_casadi = iw;

            casadi_real* w = m->w;
            m->w_casadi = w;

            m->p = malloc(sizeof(MPCpool));
            m->p->names = {prefix}p_names;
            m->p->size = {p_nominal.size};
            m->p->data = malloc(sizeof(casadi_real)*{p_nominal.size});
            m->p->n = {len(parameters)};
            m->p->trajectory_length = {prefix}p_trajectory_length;
            m->p->stride = -1;
            m->p->part_offset = {prefix}p_part_offset;
            m->p->part_unit = {prefix}p_part_unit;
            m->p->part_stride = {prefix}p_part_stride;

            m->x_current = malloc(sizeof(MPCpool));
            m->x_current->names = {prefix}x_names;
            m->x_current->size = {self.nx};
            m->x_current->data = malloc(sizeof(casadi_real)*{self.nx});
            m->x_current->n = {len(self.states)};
            m->x_current->trajectory_length = {prefix}x_current_trajectory_length;
            m->x_current->stride = {self.nx};
            m->x_current->part_offset = {prefix}x_part_offset;
            m->x_current->part_unit = {prefix}x_part_unit;
            m->x_current->part_stride = {prefix}x_part_stride;

            m->u_initial_guess = malloc(sizeof(MPCpool));
            m->u_initial_guess->names = {prefix}u_names;
            m->u_initial_guess->size = {u_nominal.size};
            m->u_initial_guess->data = malloc(sizeof(casadi_real)*{u_nominal.size});
            m->u_initial_guess->n = {len(self.controls)};
            m->u_initial_guess->trajectory_length = {prefix}u_trajectory_length;
            m->u_initial_guess->stride = {self.nu};
            m->u_initial_guess->part_offset = {prefix}u_part_offset;
            m->u_initial_guess->part_unit = {prefix}u_part_unit;
            m->u_initial_guess->part_stride = {prefix}u_part_stride;

            m->x_initial_guess = malloc(sizeof(MPCpool));
            m->x_initial_guess->names = {prefix}x_names;
            m->x_initial_guess->size = {x_nominal.size};
            m->x_initial_guess->data = malloc(sizeof(casadi_real)*{x_nominal.size});
            m->x_initial_guess->n = {len(self.states)};
            m->x_initial_guess->trajectory_length = {prefix}x_trajectory_length;
            m->x_initial_guess->stride = {self.nx};
            m->x_initial_guess->part_offset = {prefix}x_part_offset;
            m->x_initial_guess->part_unit = {prefix}x_part_unit;
            m->x_initial_guess->part_stride = {prefix}x_part_stride;

            m->u_opt = malloc(sizeof(MPCpool));
            m->u_opt->names = {prefix}u_names;
            m->u_opt->size = {u_nominal.size};
            m->u_opt->data = malloc(sizeof(casadi_real)*{u_nominal.size});
            m->u_opt->n = {len(self.controls)};
            m->u_opt->trajectory_length = {prefix}u_trajectory_length;
            m->u_opt->stride = {self.nu};
            m->u_opt->part_offset = {prefix}u_part_offset;
            m->u_opt->part_unit = {prefix}u_part_unit;
            m->u_opt->part_stride = {prefix}u_part_stride;

            m->x_opt = malloc(sizeof(MPCpool));
            m->x_opt->names = {prefix}x_names;
            m->x_opt->size = {x_nominal.size};
            m->x_opt->data = malloc(sizeof(casadi_real)*{x_nominal.size});
            m->x_opt->n = {len(self.states)};
            m->x_opt->trajectory_length = {prefix}x_trajectory_length;
            m->x_opt->stride = {self.nu};
            m->x_opt->part_offset = {prefix}x_part_offset;
            m->x_opt->part_unit = {prefix}x_part_unit;
            m->x_opt->part_stride = {prefix}x_part_stride;

            memcpy(m->p->data, {prefix}p_nominal, {p_nominal.size}*sizeof(casadi_real));
            memcpy(m->x_initial_guess->data, {prefix}x_nominal, {x_nominal.size}*sizeof(casadi_real));
            memcpy(m->u_initial_guess->data, {prefix}u_nominal, {u_nominal.size}*sizeof(casadi_real));
            memcpy(m->x_current->data, {prefix}x_current_nominal, {self.nx}*sizeof(casadi_real));
            return m;
          }}

          void {prefix}destroy(MPCstruct* m) {{
                        // Allocate memory (thread-safe)
            /* Free memory (thread-safe) */
            casadi_c_decref_id(m->id);
            // Release thread-local (not thread-safe)
            casadi_c_release_id(m->id, m->mem);
            casadi_c_pop();
            free(m);
          }}

          void {prefix}set_work(MPCstruct* m, const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w) {{
            m->arg = arg;
            m->res = res;
            m->iw = iw;
            m->w = w;
          }}

          void {prefix}work(MPCstruct* m, casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w) {{
            casadi_c_work_id(m->id, &m->sz_arg, &m->sz_res, &m->sz_iw, &m->sz_w);
            // We might want to be adding other working memory here
          }}

          int {prefix}nx(MPCstruct* m) {{

          }}

          const MPCpool* {prefix}get_pool_by_name(MPCstruct* m, const char* name) {{
            if (!strcmp(name,"p")) {{
              return m->p;
            }} else if (!strcmp(name,"x_current")) {{
              return m->x_current;
            }} else if (!strcmp(name,"x_initial_guess")) {{
              return m->x_initial_guess;
            }} else if (!strcmp(name,"u_initial_guess")) {{
              return  m->u_initial_guess;
            }} else if (!strcmp(name,"u_opt")) {{
              return  m->u_opt;
            }} else if (!strcmp(name,"x_opt")) {{
              return m->x_opt;
            }} else {{
              m->fatal(m, "get_pool_by_name", "Pool with name '%s' not recognized. \
                                       Use one of: 'p','x_initial_guess','u_initial_guess','u_opt','x_opt'. \\n", name);
              return 0;
            }}
          }}

          int {prefix}set_by_id(MPCstruct* m, const char* var_name, const char* id, const casadi_real* value) {{
            int i;
            casadi_real* dst;
            const MPCpool* p;
            p = {prefix}get_pool_by_name(m, var_name);
            if (!p) {{
              m->fatal(m, "set_by_id (ret code 1)", "Failed to find appropriate pool. \\n", id, var_name);
              return 1;
            }}
            for (i=0;i<p->n;++i) {{
              if (!strcmp(id, p->names[i])) break;
            }}
            if (i==p->n) {{
              m->fatal(m, "set_by_id (ret code 2)", "Id '%s' not found for pool '%s'. Use one of these options: \\n", id, var_name);
              for (i=0;i<p->n;++i) {{
                m->fatal(m, 0, " - %s\\n", p->names[i]);
              }}
              return 2;
            }}
            if (p->stride==-1) {{
              memcpy(p->data+p->part_offset[i], value, (p->part_offset[i+1]-p->part_offset[i])*sizeof(casadi_real));              
            }} else {{
              for (dst=p->data+p->part_offset[i];dst<p->data+p->size;dst+=p->stride) {{
                memcpy(dst, value, p->part_unit[i]*sizeof(casadi_real));
                value += p->part_unit[i];
              }}
            }}

            return 0;
          }}


          int {prefix}set_pool(const MPCpool* p, const casadi_real* value) {{
            if (!value) return 1;
            memcpy(p->data, value, p->size*sizeof(casadi_real));
            return 0;
          }}

          int {prefix}set(MPCstruct* m, const char* var_name, const casadi_real* value) {{
            return {prefix}set_pool({prefix}get_pool_by_name(m, var_name), value);
          }}

          int {prefix}get_pool(const MPCpool* p, casadi_real* value) {{
            if (!value) return 1;
            memcpy(value, p->data, p->size*sizeof(casadi_real));
            return 0;
          }}

          int {prefix}get(MPCstruct* m, const char* var_name, casadi_real* value) {{
            return {prefix}get_pool({prefix}get_pool_by_name(m, var_name), value);
          }}

          int {prefix}set_repeat(const MPCpool* p, const casadi_real* value) {{
            int i;
            casadi_real* dst;
            if (p->n==0) return 0;
            if (p->stride==-1) return 1;
            for (dst=p->data;dst<p->data+p->size;dst+=p->stride) {{
              memcpy(dst, value, p->stride*sizeof(casadi_real));
            }}
          }}

          int {prefix}set_slice(const MPCpool* p, casadi_int i, const casadi_real* value) {{
            if (p->n) return 0;
            if (p->stride==-1 || i<0 || i>=p->trajectory_length[0]) return 1;
            memcpy(p->data+i*p->stride, value, p->stride*sizeof(casadi_real));
          }}

          int {prefix}nu(MPCstruct* m) {{

          }}

          int {prefix}set_p_by_id(MPCstruct* m, const char* id, const casadi_real* value) {{
            int i;
            for (i=0;i<{len(parameters)};i++) {{
              if (p_names[i]==id) {{
                memcpy(m->p->data+{prefix}p_offsets[i], value, ({prefix}p_offsets[i+1]-{prefix}p_offsets[i])*sizeof(casadi_real));
                return 0;
              }}
            }}
            return 1;
          }}

          int {prefix}print_problem(MPCstruct* m) {{
            int i,j,k,l,max_len;
            const MPCpool* p;
            max_len=0;
            for (l=0;l<4;++l) {{
              p = get_pool_by_name(m, {prefix}pool_names[l]);
              for (i=0;i<p->n;++i) {{
                max_len = strlen(p->names[i])>max_len? strlen(p->names[i]) : max_len;
              }}
            }}

            if (m->fp) {{
              for (l=0;l<4;++l) {{
                const MPCpool* p = get_pool_by_name(m, {prefix}pool_names[l]);
                m->info(m, "=== %s ===\\n", {prefix}pool_names[l]);
                char formatbuffer[10];
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
          }}

          int {prefix}solve(MPCstruct* m) {{
            int i;
            m->arg_casadi[0] = m->x_initial_guess->data;
            m->arg_casadi[1] = m->u_initial_guess->data;
            for (i=0;i<{len(parameters)};i++) {{
              m->arg_casadi[2+i] = m->p->data + {prefix}p_offsets[i];
            }}
            m->res_casadi[0] = m->x_opt->data;
            m->res_casadi[1] = m->u_opt->data;
            //return casadi_c_eval_id(m->id, m->arg_casadi, m->res_casadi, m->iw_casadi, m->w_casadi, m->mem);
          }}

          int {prefix}get_p_by_id(MPCstruct* m, const char* id, const casadi_real* value) {{

          }}

          void {prefix}get_u(MPCstruct* m, double* value) {{

          }}

        """)

    lib_name = name
    lib_file_name = os.path.join(build_dir_abs,"lib" + lib_name + ".so")

    import subprocess
    subprocess.Popen(["gcc","-g","-fPIC","-shared",c_file_name,"-lcasadi","-L"+GlobalOptions.getCasadiPath(),"-I"+GlobalOptions.getCasadiIncludePath(),"-o"+lib_file_name,"-Wl,-rpath="+GlobalOptions.getCasadiPath()]).wait()
    args = ["gcc","-g",hello_c_file_name,"-I"+build_dir_abs,"-l"+lib_name,"-L"+build_dir_abs,"-o",hello_file_name,"-Wl,-rpath="+build_dir_abs]
    print(" ".join(args))
    subprocess.Popen(args).wait()
      
    """
    #define S_FUNCTION_NAME  casadi_fun
#define S_FUNCTION_LEVEL 2

#include <casadi/casadi_c.h>

#include "simstruc.h"

static int id = -1;
static int ret = -1;

static casadi_int n_in, n_out;
static casadi_int sz_arg, sz_res, sz_iw, sz_w;

static int mem;


void cleanup() {
  if (ret==0) {
    casadi_c_pop();
    ret = -1;
  }
}

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;
    const casadi_int* sp;
    const char *file_name;
    const char *function_name;
    ssSetNumSFcnParams(S, 2);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!mxIsChar(ssGetSFcnParam(S, 0))) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "file name must be a string.");
    }
    if (!mxIsChar(ssGetSFcnParam(S, 1))) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "function name must be a string.");
    }
    
    file_name = mxArrayToString(ssGetSFcnParam(S, 0));

    if (!file_name) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "file name must be a string.");
    }
    function_name = mxArrayToString(ssGetSFcnParam(S, 1));
    if (!function_name) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:invalidParameter",
                         "function name must be a string.");
    }

    // Simulink does not provide a cleanup-hook when parameters are changed
    cleanup();

    // Load file
    mexPrintf("Loading file '%s'...", file_name);
    ret = casadi_c_push_file(file_name);
    mxFree(file_name);

    if (ret) {
      mexErrMsgIdAndTxt( "MATLAB:s_function:Load",
                         "Failed to load file.");
    }
    mexPrintf("success\n");

    // Load function
    mexPrintf("Locating function '%s'...", function_name);
    id = casadi_c_id(function_name);
    mxFree(function_name);
    if (id<0) {
      casadi_c_pop();
      mexErrMsgIdAndTxt( "MATLAB:s_function:Load",
                         "Failed to locate function in loaded file.");
    }
    mexPrintf("success\n");


    /* Read in CasADi function dimensions */
    n_in = casadi_c_n_in_id(id);
    n_out = casadi_c_n_out_id(id);
    casadi_c_work_id(id, &sz_arg, &sz_res, &sz_iw, &sz_w);
    
    /* Set up simulink input/output ports */
    if (!ssSetNumInputPorts(S, n_in)) return;
    for (i=0;i<n_in;++i) {
       sp = casadi_c_sparsity_in_id(id, i);
      /* Dense inputs assumed here */
      ssSetInputPortDirectFeedThrough(S, i, 1);
      casadi_int nnz = sp[2+sp[1]];
      if (nnz!=sp[0]*sp[1]) {
        casadi_c_pop();
        mexErrMsgIdAndTxt( "MATLAB:s_function:sparsity",
                         "This example only supports dense inputs.");
      }
      ssSetInputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }

    if (!ssSetNumOutputPorts(S, n_out)) return;
    for (i=0;i<n_out;++i) {
      sp = casadi_c_sparsity_out_id(id, i);
      casadi_int nnz = sp[2+sp[1]];
      /* Dense outputs assumed here */
      if (nnz!=sp[0]*sp[1]) {
        casadi_c_pop();
        mexErrMsgIdAndTxt( "MATLAB:s_function:sparsity",
                         "This example only supports dense outputs. Use 'densify'.");
      }
      ssSetOutputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }

    ssSetNumSampleTimes(S, 1);
    
    /* Set up CasADi function work vector sizes */
    ssSetNumRWork(S, sz_w);
    ssSetNumIWork(S, sz_iw*sizeof(casadi_int)/sizeof(int_T));
    ssSetNumPWork(S, sz_arg+sz_res);
    ssSetNumNonsampledZCs(S, 0);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    // Make sure mdlTerminate is called on error
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    void** p;
    const real_T** arg;
    double* w;
    casadi_int* iw;
    int_T i;

    /* Set up CasADi function work vectors */
    p = ssGetPWork(S);
    arg = (const real_T**) p;
    p += sz_arg;
    real_T** res = (real_T**) p;
    w = ssGetRWork(S);
    iw = (casadi_int*) ssGetIWork(S);
    
    
    /* Point to input and output buffers */  
    for (i=0; i<n_in;++i) {
      arg[i] = *ssGetInputPortRealSignalPtrs(S,i);
    }
    for (i=0; i<n_out;++i) {
      res[i] = ssGetOutputPortRealSignal(S,i);
    }

    /* Run the CasADi function */
    if (casadi_c_eval_id(id, arg, res, iw, w, mem)) {
      ssPrintf("Failed to evaluate\n");
    }
}

static void mdlStart(SimStruct *S)
{
    // Allocate memory (thread-safe)
    casadi_c_incref_id(id);
    // Checkout thread-local memory (not thread-safe)
    mem = casadi_c_checkout_id(id);
}

static void mdlTerminate(SimStruct *S) {
  /* Free memory (thread-safe) */
  casadi_c_decref_id(id);
  // Release thread-local (not thread-safe)
  casadi_c_release_id(id, mem);

  cleanup();
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
"""