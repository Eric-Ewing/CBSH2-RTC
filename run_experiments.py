from glob import glob
import numpy as np
from tqdm.rich import tqdm
import re
from subprocess import Popen, PIPE

fails_dict = {}
counter = 0
up_to_date = False


def extract_args(command):
    scen = command.split('-a')[1].split(' ')[1]
    k = command.split('-k')[1].split(' ')[1]
    rr = command.split('--heuristics=')[1].split('\n')[0]
    cr = command.split('--corridorReasoning=')[1].split('\n')[0]
    sipp = command.split('--sipp=')[1]
    decomp = command.split('--decompose=')[1].split(' ')[0]
    theta = command.split('--threshold=')[1].split(' ')[0]

    return scen, k, rr, cr, sipp, decomp, theta

def get_prev_command(command):
    k_value = re.search(r'-k (\d{1,3})', command)
    if k_value:
        k = int(re.search(r'-k (\d{1,3})', command).group(1))
        new_k = k - 5
        new_command = re.sub(r'-k (\d{1,3})', f'-k {new_k}', command)
        return new_command
    return None

def execute_command(command):
    with open('thesis_experiments/successful_experiments.txt', 'r') as f:
        succesful_experiments = f.read()
    with open('thesis_experiments/failed_experiments.txt', 'r') as f:
        failed_experiments = f.read()
    
    if command in succesful_experiments or command in failed_experiments:
        return
    prev_command = get_prev_command(command)
    if prev_command in failed_experiments:
        return
    global fails_dict
    # Check if 10 fails for any of previous n agents...
    # scen, k, rr, cr, sipp, decomp, theta = extract_args(command)
    # prev_agents = int(k) - 5
    # args_str = ''.join([scen, str(prev_agents), rr, cr, sipp, decomp, theta])
    # if args_str in fails_dict:
    #     if fails_dict[args_str] >= 1:
    #         print(command)
    #         fails_dict[''.join([scen, k, rr, cr, sipp, decomp, theta])] = 1
    #         print('exit!')
    #         return
    # print(command)
    p = Popen('timeout 320s ' + command, shell=True, stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()
    if not ('Optimal' in str(out)):
        # key = ''.join([scen, k, rr, cr, sipp, decomp, theta])
        # if key not in fails_dict:
            # fails_dict[key] = 0
        # fails_dict[key] += 1
        # print('failed!')
        # print(command)
        with open('thesis_experiments/failed_experiments.txt', 'a') as f:
            f.write(command + '\n')
    else:
        with open('thesis_experiments/successful_experiments.txt', 'a') as f:
            f.write(command + '\n')



agents = range(40, 150, 5)
rectangular_reasoning = ["WDG"]
corridor_reasoning = ["GC"]
# use_sipp = [True, False]
use_sipp = [False]
maps = glob('socs_maps/*.map')
# maps = ["maps/empty_210.map"]
scens = glob('scens/*.scen')
# scens = glob('python/unif_scen/*.scen')
decompose = ["true", "false"]
# decompose = ["false"]
thresholds = np.arange(0.0, 0.3, 0.1)
experiments_filename = "thesis_experiments/cbsh.csv"
commands = []

for a in agents:
    for r in rectangular_reasoning:
        for c in corridor_reasoning:
            for m in maps:
                for s in scens:
                    for sipp in use_sipp:
                        sipp = int(sipp)
                        map_name = m.split('/')[-1].split('.')
                        if (map_name[0] not in s):
                            # print(m, s)
                            continue
                        for d in decompose:
                            if d == "true":
                                for t in thresholds:
                                    command = f"./cbs -m {m} -a {s} -o {experiments_filename} -k {a} -t 100 --decompose={d} --heuristics={r} --corridorReasoning={c} --threshold={t} --sipp={sipp}"
                                    commands.append(command) 
                                    # command = f"./cbs -m {m} -a {s} -o {experiments_filename} -k {a} -t 300 --decompose={d} --heuristics={r} --corridorReasoning={c} --threshold={t} --sipp={sipp}"
                                    # commands.append(command)
                            else:
                                command = f"./cbs -m {m} -a {s} -o {experiments_filename} -k {a} -t 100 --decompose={d} --heuristics={r} --corridorReasoning={c} --sipp={sipp}"
                                # command = f"./cbs -m maps/empty_210.map -a {s} -o {experiments_filename + str(len(commands))} -k {a} -t 30 --decompose=false --heuristics=Zero --corridorReasoning=None --rectangleReasoning=None --threshold=0.1 --sipp={sipp}"
                                commands.append(command)

with open('failed_experiments.txt', 'r') as f:
    try:
        while True:
            command = f.readline()
            scen, k, rr, cr, sipp, decomp, theta = extract_args(command)
            key = ''.join([scen, k, rr, cr, sipp, decomp, theta])
            if key not in fails_dict:
                fails_dict[key] = 0
            fails_dict[key] += 1
    except:
        pass

# print(len(commands))
for c in tqdm(commands):
    execute_command(c)
