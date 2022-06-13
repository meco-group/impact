classdef MPC < rockit.Ocp & rockit.Stage
  properties
  end
  methods
    function obj = MPC(varargin)
      obj@rockit.Ocp('from_super');
      obj@rockit.Stage('from_super');
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.dirac_mpc.mpc.MPC')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = dirac_mpc.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'kwargs'});
      if isempty(kwargs)
        obj.parent = py.dirac_mpc.MPC(args{:});
      else
        obj.parent = py.dirac_mpc.MPC(args{:},pyargs(kwargs{:}));
      end
    end
    function varargout = parameter(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,-inf,{'name','args','kwargs'});
      if isempty(kwargs)
        res = obj.parent.parameter(args{:});
      else
        res = obj.parent.parameter(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = variable(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,-inf,{'name','args','kwargs'});
      if isempty(kwargs)
        res = obj.parent.variable(args{:});
      else
        res = obj.parent.variable(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = add_function(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'fun'});
      if isempty(kwargs)
        res = obj.parent.add_function(args{:});
      else
        res = obj.parent.add_function(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = add_model(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,2,{'name','file_name'});
      if isempty(kwargs)
        res = obj.parent.add_model(args{:});
      else
        res = obj.parent.add_model(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_x(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_x(args{:});
      else
        res = obj.parent.opti_x(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_f(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_f(args{:});
      else
        res = obj.parent.opti_f(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_g(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_g(args{:});
      else
        res = obj.parent.opti_g(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_lbg(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_lbg(args{:});
      else
        res = obj.parent.opti_lbg(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_ubg(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_ubg(args{:});
      else
        res = obj.parent.opti_ubg(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_lam_g(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_lam_g(args{:});
      else
        res = obj.parent.opti_lam_g(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = opti_p(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.opti_p(args{:});
      else
        res = obj.parent.opti_p(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = export(obj,varargin)

    varargin = [varargin {'context','matlab'}];
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','src_dir','use_codegen','context','ignore_errors','short_output'});
      if isempty(kwargs)
        res = obj.parent.export(args{:});
      else
        res = obj.parent.export(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);

    name = varargin{1};
    build_dir = [pwd filesep '' name '_build_dir'];
    addpath(build_dir);
    current = cd(build_dir);
    run('build.m');
    cd(current);
    end
    function out = expr(obj)
      global pythoncasadiinterface
      out = pythoncasadiinterface.python2matlab(obj.parent.expr);
    end
    function out = patch_codegen(obj)
      % staticmethod(function) -> method
      % 
      % Convert a function to be a static method.
      % 
      % A static method does not receive an implicit first argument.
      % To declare a static method, use this idiom:
      % 
      %      class C:
      %          @staticmethod
      %          def f(arg1, arg2, ...):
      %              ...
      % 
      % It can be called either on the class (e.g. C.f()) or on an instance
      % (e.g. C().f()).  The instance is ignored except for its class.
      % 
      % Static methods in Python are similar to those found in Java or C++.
      % For a more advanced concept, see the classmethod builtin.
      global pythoncasadiinterface
      out = pythoncasadiinterface.python2matlab(obj.parent.patch_codegen);
    end
  end
end
