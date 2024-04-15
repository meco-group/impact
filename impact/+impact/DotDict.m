classdef DotDict < handle
  % like a dict but access through . 
  properties
    parent
  end
  methods
    function obj = DotDict(varargin)
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.impact.dotdict.DotDict')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = impact.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'d'});
      if isempty(kwargs)
        obj.parent = py.impact.DotDict(args{:});
      else
        obj.parent = py.impact.DotDict(args{:},pyargs(kwargs{:}));
      end
    end
    function varargout = subsref(obj,S)
      if ~strcmp(S(1).type,'.')
        [varargout{1:nargout}] = builtin('subsref',obj,S);
        return
      end
      varargin = {S(1).subs};
      callee = py.getattr(obj.parent,'__getattr__');
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,1,{'k'});
      if isempty(kwargs)
        res = callee(args{:});
      else
        res = callee(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
      if numel(varargout) == 1 && numel(S)>1
        varargout = {subsref(varargout{1},S(2:end))};
      end
    end
    function varargout = disp(obj)
      varargin = {};
      callee = py.getattr(obj.parent,'__repr__');
      global pythoncasadiinterface
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'indent'});
      if isempty(kwargs)
        res = callee(args{:});
      else
        res = callee(args{:},pyargs(kwargs{:}));
      end
      varargout = pythoncasadiinterface.python2matlab_ret(res);
      disp(varargout{1});
      varargout = {};
    end
  end
end
