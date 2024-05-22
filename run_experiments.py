from glob import glob
import numpy as np
from tqdm.rich import tqdm

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


def execute_command(command):
    global up_to_date
    # if not up_to_date:
    #     if command == './cbs -m socs_maps/Berlin_1_256.map -a scens/Berlin_1_256-even-8.scen -o experiments/socs_experiments_v6.csv -k 45 -t 300 --decompose=true --heuristics=WDG --corridorReasoning=GC --threshold=0.1 --sipp=1':
    #         up_to_date = True
    #         print('match')
    #         return
    #     else:
    #         return
    # if 'maze' in command:
    #     return

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
        key = ''.join([scen, k, rr, cr, sipp, decomp, theta])
        if key not in fails_dict:
            fails_dict[key] = 0
        fails_dict[key] += 1
        print('failed!')
        print(command)
        with open('failed_experiments.txt', 'a') as f:
            f.write(command + '\n')


agents = range(5, 120, 5)
rectangular_reasoning = ["WDG"]
corridor_reasoning = ["GC"]
# use_sipp = [True, False]
use_sipp = [False]
# maps = glob('socs_maps/*.map')
maps = ["maps/empty_210.map"]
# scens = glob('scens/*.scen')
scens = glob('python/unif_scen/*.scen')
# decompose = ["true", "false"]
decompose = ["false"]
thresholds = np.arange(0.0, 0.3, 0.1)
experiments_filename = "empty_uniform_experiments/uniform_random.csv"
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
                                    command = f"./cbs -m {m} -a {s} -o {experiments_filename} -k {a} -t 300 --decompose={d} --heuristics={r} --corridorReasoning={c} --threshold={t} --sipp={sipp}"
                                    commands.append(command) 
                                    # command = f"./cbs -m {m} -a {s} -o {experiments_filename} -k {a} -t 300 --decompose={d} --heuristics={r} --corridorReasoning={c} --threshold={t} --sipp={sipp}"
                                    # commands.append(command)
                            else:
                                # command = f"./cbs -m {m} -a {s} -o {experiments_filename + str(len(commands))} -k {a} -t 300 --decompose={d} --heuristics={r} --corridorReasoning={c} --sipp={sipp}"
                                command = f"./cbs -m maps/empty_210.map -a {s} -o {experiments_filename + str(len(commands))} -k {a} -t 30 --decompose=false --heuristics=Zero --corridorReasoning=None --rectangleReasoning=None --threshold=0.1 --sipp={sipp}"
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

print(len(commands))
for c in tqdm(commands):
    execute_command(c)
