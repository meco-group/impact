classdef Impact < handle
  properties
    parent
  end
  methods
    function obj = Impact(varargin)
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.dirac_mpc.impact.Impact')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = dirac_mpc.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'name','dir'});
      if isempty(kwargs)
        obj.parent = py.dirac_mpc.Impact(args{:});
      else
        obj.parent = py.dirac_mpc.Impact(args{:},pyargs(kwargs{:}));
      end
    end
    function varargout = get(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,4,{'pool_name','id','stage','flags'});
      if isempty(kwargs)
        res = obj.parent.get(args{:});
      else
        res = obj.parent.get(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = get_size(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,2,{'pool_name','id'});
      if isempty(kwargs)
        res = obj.parent.get_size(args{:});
      else
        res = obj.parent.get_size(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = print_problem(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.print_problem(args{:});
      else
        res = obj.parent.print_problem(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
    function varargout = solve(obj,varargin)
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{});
      if isempty(kwargs)
        res = obj.parent.solve(args{:});
      else
        res = obj.parent.solve(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
    end
  end
end
