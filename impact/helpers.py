import os



def simulink_parse_states(mdl,unzipped_path):

    def get_full_id(id):
        with open(os.path.join(unzipped_path,'sources',id+"_gateway.c"),'r') as inp:
            for line in inp:
                line = line.strip()
                if "_".join(id.split("_")[:-1]) in line:
                    return line.split('"')[1][:-2]

    def get_state_names(full_id):
        with open(os.path.join(unzipped_path,'sources',full_id+"_ds.c"),'r') as inp:
            harvesting = False
            harvest = ""
            for line in inp:
                line = line.strip()
                if "NeVariableData s_variable_data" in line:
                    harvesting = True
                    harvest = line
                elif harvesting:
                    harvest += line
                    if ";" in line:
                        break



        harvest = harvest[harvest.index("{"):harvest.rindex("}")+1]

        from pyparsing import nestedExpr, delimitedList
        content = nestedExpr(opener="{", closer="}", content=delimitedList(nestedExpr(opener="{", closer="}")))
        harvest = content.parseString(harvest,parseAll=True).asList()[0]
        names = []
        for e in harvest:
            names.append(e[0][1:-1])

        return names

    state_class = 'X_'+mdl+'_T'

    typedefs = {}

    typedef_staging = None
    prev_line = None
    simscape_states = []
    simscape_ids = []

    with open(os.path.join(unzipped_path,'sources',mdl+'.c'),'r') as inp:
        for line in inp:
            line = line.strip()
            if "_gateway(" in line:
                active_simscape = line[:line.index("(")].rstrip()
                if active_simscape not in simscape_ids:
                    simscape_ids.append(active_simscape[:-len("_gateway")])
            if "simulationData->mData->mContStates.mX" in line and "NULL" not in line:
                n = int(prev_line.split("=")[1][:-1])
                name = line.split("->")[-1].split("[")[0]
                if (name,n) not in simscape_states:
                    simscape_states.append((name,n))
            prev_line = line

    simscape_state = {}

    for k in range(len(simscape_states)):
        short_id = simscape_ids[k]
        full_id = get_full_id(short_id)
        names = get_state_names(full_id)
        assert len(names)==simscape_states[k][1]
        simscape_state[simscape_states[k][0]] = names

    print(simscape_state)

    with open(os.path.join(unzipped_path,'sources',mdl+'.h'),'r') as inp:
        for line in inp:
            line = line.strip()
            if "typedef struct" in line:
                typedef_staging = []
            else:
                if typedef_staging is not None:
                    if "}" in line:
                        typedefs[line[1:-1].strip()] = typedef_staging
                        typedef_staging = None
                    else:
                        line = line[:-1]
                        [t,name] = line.split(" ")
                        typedef_staging.append((t,name))
    
    states = typedefs[state_class]

    for e in states:
        assert e[0]=='real_T'

    states = [e[1] for e in states]

    all_states = []
    for e in states:
        if "[" in e:
            name = e[:e.index("[")]
            index = int(e[e.index("[")+1:e.index("]")])
            assert name in simscape_state
            assert index==len(simscape_state[name])
            all_states+=simscape_state[name]
        else:
            all_states.append(e)

    return all_states