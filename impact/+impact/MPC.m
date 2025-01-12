classdef MPC < rockit.Ocp & rockit.Stage
  % This should be a description of the MPC class.
  %   It's common for programmers to give a code example inside of their
  %   docstring::
  % 
  %       from impact import MPC
  %       mpc = MPC(T=2.0)
  % 
  %   Here is a link to :py:meth:`__init__`.
  %   
  properties
  end
  methods
    function obj = MPC(varargin)
      % Inits MPC class.
      % Arguments: kwargs
      obj@rockit.Ocp('from_super');
      obj@rockit.Stage('from_super');
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.impact.mpc.MPC')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = impact.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'kwargs'});
      if isempty(kwargs)
        obj.parent = py.impact.MPC(args{:});
      else
        obj.parent = py.impact.MPC(args{:},pyargs(kwargs{:}));
      end
    end
    function varargout = control(obj,varargin)
      % Defines control variable
      % Arguments: args, kwargs
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,-inf,{'name','args','kwargs'});
      if isempty(kwargs)
        res = obj.parent.control(args{:});
      else
        res = obj.parent.control(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = state(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,-inf,{'name','args','kwargs'});
      if isempty(kwargs)
        res = obj.parent.state(args{:});
      else
        res = obj.parent.state(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
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
    function varargout = add_simulink_fmu(obj,varargin)
      % 
      % Arguments: name, verbose=True
      %       Not supported: 
      %       * time dependence
      %       * delays
      %       Perhpas SS is better
      %       Caveats:
      %       * scaling for finite diff
      %     
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','verbose'});
      if isempty(kwargs)
        res = obj.parent.add_simulink_fmu(args{:});
      else
        res = obj.parent.add_simulink_fmu(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = add_model(obj,varargin)
      % Creates a model based on a yaml file
      % Arguments: name, file_name
      % 
      %         :param name: Name of the model
      %         :type name: string
      % 
      %         :param file_name: Path to the yaml file
      %         :type name: string
      % 
      %         :return: model
      %         :rtype: Model
      % 
      %     
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
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','src_dir','use_codegen','context','ignore_errors','short_output','qp_error_on_fail','c_flags'});
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
    function varargout = save(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name'});
      if isempty(kwargs)
        res = obj.parent.save(args{:});
      else
        res = obj.parent.save(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function out = expr(obj)
      global pythoncasadiinterface
      out = pythoncasadiinterface.python2matlab(obj.parent.expr);
    end
  end
  methods(Static)
    function varargout = patch_codegen(varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','ocp','qp_error_on_fail'});
      if isempty(kwargs)
        res = py.impact.mpc.MPC.patch_codegen(args{:});
      else
        res = py.impact.mpc.MPC.patch_codegen(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = load(varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'name'});
      if isempty(kwargs)
        res = py.impact.mpc.MPC.load(args{:});
      else
        res = py.impact.mpc.MPC.load(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
  end
end
