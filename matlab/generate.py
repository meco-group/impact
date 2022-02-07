import python_matlab
import dirac_mpc

exclude = []
custom = {}
callback_post = {}
callback_post[r"MPC\.export"] = f"""
    name = varargin{{1}};
    build_dir = [pwd filesep '{dirac_mpc.build_dir_prefix}' name '{dirac_mpc.build_dir_suffix}'];
    addpath(build_dir);
    current = cd(build_dir);
    run('build.m');
    cd(current);
"""
callback_pre = {}
callback_pre[r"MPC\.export"] = f"""
    varargin = [varargin {{'context','matlab'}}];
"""
python_matlab.generate(dirac_mpc,exclude=exclude,custom=custom,callback_post=callback_post,callback_pre=callback_pre)
