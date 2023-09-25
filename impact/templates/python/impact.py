from ctypes import *
import numpy as np
import os

try:
    import casadi
    os.environ["PATH"] += os.pathsep + casadi.GlobalOptions.getCasadiPath()
except:
    pass

class Impact:
    def _register(self,fun_name,argtypes,restype):
        fun = getattr(self.lib,self.prefix+fun_name)
        setattr(self,"_"+fun_name,fun)
        fun.argtypes = argtypes
        fun.restype = restype

    def __init__(self, name, src_dir="."):
        self.prefix = "impact_"
        build_dir_rel = name+"_build_dir"
        build_dir_abs = os.path.join(os.path.abspath(src_dir),build_dir_rel)

        # PyDLL instead of CDLL to keep GIL:
        # virtual machine emits Python prints
        if os.name == "nt":
            libname = name+".dll"
        else:
            libname = "lib"+name+".so"
        self.lib = PyDLL(os.path.join(build_dir_abs,libname))

        # Type aliases
        m_type = c_void_p
        CONST = lambda x: x

        # Relay printing to python
        formatter_type = CFUNCTYPE(c_int, c_char_p)
        def formatter(c):
            print(c.decode('ascii'),end='')
            return 0
        self._formatter = formatter_type(formatter)

        self._register("initialize",[formatter_type, c_char_p], m_type)
        self._register("destroy",[m_type], None)
        self._register("get",[m_type, c_char_p, c_char_p, c_int, POINTER(c_double), c_int], c_int)
        self._register("set",[m_type, c_char_p, c_char_p, c_int, POINTER(CONST(c_double)), c_int], c_int)
        self._register("print_problem",[m_type], c_int)
        self._register("get_id_count",[m_type, c_char_p], c_int)
        self._register("get_size",[m_type, c_char_p, c_char_p, c_int, c_int, POINTER(c_int), POINTER(c_int)], c_int)
        self._register("solve",[m_type], c_int)
        self._register("hotstart",[m_type], c_int)
        self._register("flag_size",[m_type], c_int)
        self._register("flag_name",[m_type, c_int], c_char_p)
        self._register("flag_value",[m_type, c_int], c_int)

        self._m = self._initialize(self._formatter, build_dir_abs.encode("ascii"))
        """
          int {prefix}get_id_count(MPCstruct* m, const char* pool_name);
          int {prefix}get_id_from_index(MPCstruct* m, const char* pool_name, int index, const char** id);
          int {prefix}get_size(MPCstruct* m, const char* pool_name, const char* id, int* n_row, int* n_col);

          int {prefix}print_problem(MPCstruct* m);
          void {prefix}set_work(MPCstruct* m, const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w);
          void {prefix}work(MPCstruct* m, casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);

        # MPCstruct* m = initialize(printf)
        """

        for i in range(self._flag_size(self._m)):
            setattr(self, self._flag_name(self._m, i).decode("ascii"), self._flag_value(self._m, i))

    def get(self, pool_name, id, stage, flags):
        shape = self.get_size(pool_name, id, stage, flags)
        ret = np.zeros(shape,dtype=np.float64)
        if 1 not in shape:
            flags |= self.ROW_MAJOR
        self._get(self._m, pool_name.encode("ascii"), None if id==self.ALL else id.encode("ascii"), stage, ret.ctypes.data_as(POINTER(c_double)), flags)
        return ret

    def set(self, pool_name, id, stage, flags, data):
        if isinstance(data,int) or  isinstance(data,float):
          data = np.array([data],dtype=float)
        elif isinstance(data,list):
          data = np.array(data,dtype=float)
        shape = self.get_size(pool_name, id, stage, flags)
        if len(data.shape)==1 and 1 in shape:
            pass
        elif data.shape==shape:
            flags |= self.ROW_MAJOR
        else:
            raise Exception(f"Expected shape {shape}, got {data.shape} instead.")
        self._set(self._m, pool_name.encode("ascii"), None if id==self.ALL else id.encode("ascii"), stage, data.ctypes.data_as(POINTER(c_double)), flags)

    def get_size(self, pool_name, id, stage, flags):
        row = c_int()
        col = c_int()
        self._get_size(self._m, pool_name.encode("ascii"), None if id==self.ALL else id.encode("ascii"), stage, flags, byref(row), byref(col))
        return (row.value, col.value)

    def print_problem(self):
        self._print_problem(self._m)

    def solve(self):
        self._solve(self._m)

    def hotstart(self):
        self._hotstart(self._m)

    def __del__(self):
        if hasattr(self, "_m"):
            self._destroy(self._m)
