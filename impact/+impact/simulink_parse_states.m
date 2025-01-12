function varargout = simulink_parse_states(varargin)
  global pythoncasadiinterface
    if isempty(pythoncasadiinterface)
    pythoncasadiinterface = impact.PythonCasadiInterface;
  end
  [args,kwargs] = pythoncasadiinterface.matlab2python_arg(varargin,2,{'mdl','unzipped_path'});
  if isempty(kwargs)
    res = py.impact.simulink_parse_states(args{:});
  else
    res = py.impact.simulink_parse_states(args{:},pyargs(kwargs{:}));
  end
  varargout = pythoncasadiinterface.python2matlab_ret(res);
end
