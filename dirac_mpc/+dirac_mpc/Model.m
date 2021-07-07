classdef Model < dirac_mpc.DotDict
  properties
  end
  methods
    function obj = Model(varargin)
      obj@dirac_mpc.DotDict('from_super');
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.dirac_mpc.mpc.Model')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = dirac_mpc.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'prefix'});
      if isempty(kwargs)
        obj.parent = py.dirac_mpc.Model(args{:});
      else
        obj.parent = py.dirac_mpc.Model(args{:},pyargs(kwargs{:}));
      end
    end
  end
end
