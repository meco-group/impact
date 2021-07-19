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
    function varargout = export(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','src_dir','use_codegen'});
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
  end
end
