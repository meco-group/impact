function [] = export_simulink_model(mdl, export_filename)
    load_system(mdl)
    mdl_init = [mdl '_init'];
    evalin('base', mdl_init);

    cs = getActiveConfigSet(gcs);

    cs.set_param('SolverType', 'Fixed-step');
    cs.set_param('StartTime', '0.0');
    cs.set_param('FixedStep', num2str(0.1)); 
    cs.set_param('Solver', 'ode4');

    exportToFMU(mdl,'FMIVersion','2.0','FMUType','ME','SaveSourceCodeToFMU','on','SupportMultiInstance','on', 'SaveDirectory',pwd);
    unzipped_path = [mdl '_unzipped'];

    %if exist(unzipped_path)
    %    rmdir(unzipped_path, 's');
    %end

    unzip([mdl '.fmu'],unzipped_path)

    dae = casadi.DaeBuilder(mdl, unzipped_path)

    par_names = cellstr(dae.p());
    par_values = dae.start(dae.p());

    % from Simulink model collect all states ( ContinuousStateAttributes State Name)
    
    input_names = dae.u();
    if isempty(input_names)
        input_names = {};
    else
        input_names = cellstr(input_names);
    end
    parameter_names = dae.p();
    if isempty(parameter_names)
        parameter_names = {};
    else
        parameter_names = cellstr(parameter_names);
    end
    % Simulink.fmuexport.GetExportableVariableList(mdl,'output','flat'); % contains no dimension info
    output_names = dae.y();
    if isempty(output_names)
        output_names = {};
    else
        output_names = cellstr(output_names);
    end

    state_names = impact.simulink_parse_states(mdl,unzipped_path);

    %initial_state = Simulink.BlockDiagram.getInitialState(mdl);

    %state_names = {};

    %for i=1:initial_state.numElements
    %    state_names{end+1} = initial_state{i}.Name;
    %end



    % Open export_filename for writing
    fid = fopen(export_filename, 'w');

    fprintf(fid, 'equations:\n');
    fprintf(fid, '  external:\n');
    fprintf(fid, '    type: fmu\n');
    fprintf(fid, '    file_name: %s\n', [mdl '.fmu']);


    if ~isempty(state_names)
        fprintf(fid, 'differential_states:\n');
        for i=1:length(state_names)
            fprintf(fid, '  - name: %s\n', state_names{i});
        end
    end
    if ~isempty(input_names)
        fprintf(fid, 'controls:\n');
        for i=1:length(input_names)
            fprintf(fid, '  - name: %s\n', input_names{i});
        end
    end
    if ~isempty(output_names)
        fprintf(fid, 'outputs:\n');
        for i=1:length(output_names)
            fprintf(fid, '  - name: %s\n', output_names{i});
        end
    end
    if ~isempty(parameter_names)
        fprintf(fid, 'parameters:\n');
        for i=1:length(parameter_names)
            assert(strcmp(parameter_names{i},par_names{i}))
            fprintf(fid, '  - name: %s\n', parameter_names{i});
            fprintf(fid, '    value: %.18e\n', par_values(i));
        end
    end
    fclose(fid);
end
