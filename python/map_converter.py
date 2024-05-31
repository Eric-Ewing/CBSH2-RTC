from glob import glob
import yaml

for map_file in glob('../socs_maps/*'):
    with open(map_file, 'r') as f:
        map = f.read()
    free_space = []
    obstacles = []
    edited_map = map.split('\n')[4:]
    for i, l in enumerate(edited_map):
        for j, c in enumerate(l):
            if c == '.' or c == 'G':
                free_space.append((j, i))
            else:
                obstacles.append((i, j))
    max_x = max([x[1] for x in free_space])
    max_y = max([y[0] for y in free_space])
    output = {}
    dimensions = [max_x+1, max_y+1]
    output['obstacles'] = obstacles
    output['dimensions'] = dimensions
    out_file = map_file.split('/')[-1]
    with open(f'../yaml_maps/{out_file}', 'w') as f:
        yaml.safe_dump(output, f)
    
    for scen_file in glob('../scens/*'):
        if map_file.split('/')[2].split('.')[0] in scen_file:
            with open(scen_file, 'r') as f:
                scen = f.read()
            agents = scen.split('\n')[1:]
            for i in range(40, 200, 5):
                agents_in_instance = agents[:i]
                out_agents = []
                for j, a in enumerate(agents_in_instance):
                    props = a.split('\t')
                    if len(props) <= 2:
                        continue
                    start = (int(props[4]), int(props[5]))
                    goal = (int(props[6]), int(props[7]))
                    name = 'agent{}'.format(j) 
                    out_agents.append({'name': name, 'start': start, 'goal': goal})
                with open('../yaml_instances/' + scen_file.split('.')[-2].split('/')[-1] +'_'+str(i)+'.map', 'w') as f:
                    print(scen_file.split('.')[-2].split('/')[-1] +'_'+str(i)+'.map')
                    yaml.safe_dump({'agents': out_agents, 'map_path': f'../yaml_maps/{out_file}'}, f)