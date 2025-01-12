import os

def simulink_parse_states(mdl,unzipped_path):

    state_class = 'X_'+mdl+'_T'

    typedefs = {}

    typedef_staging = None

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
                        [type,name] = line.split(" ")
                        typedef_staging.append((type,name))
    
    states = typedefs[state_class]

    for e in states:
        assert e[0]=='real_T'

    states = [e[1] for e in states]

    return states