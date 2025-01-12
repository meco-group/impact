classdef Model < impact.DotDict
  % This should be a description of the Model class.
  properties
  end
  methods
    function obj = Model(varargin)
      obj@impact.DotDict('from_super');
      if length(varargin)==1 && ischar(varargin{1}) && strcmp(varargin{1},'from_super'),return,end
      if length(varargin)==1 && isa(varargin{1},'py.impact.model.Model')
        obj.parent = varargin{1};
        return
      end
      global pythoncasadiinterface
      if isempty(pythoncasadiinterface)
        pythoncasadiinterface = impact.PythonCasadiInterface;
      end
      [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,0,{'prefix'});
      if isempty(kwargs)
        obj.parent = py.impact.Model(args{:});
      else
        obj.parent = py.impact.Model(args{:},pyargs(kwargs{:}));
      end
    end
  end
end
