from glob import glob
import os
import numpy as np
import multiprocessing
from tqdm import tqdm

agents = range(5, 205, 5)
rectangular_reasoning = ["WDG", "CG"]
corridor_reasoning = ["GC", "Disjoint"]
maps = glob('maps/*.map')
scens = glob('scens/*.scen')
decompose = ["true", "false"]
thresholds = np.arange(0.0, 0.5, 0.1)
experiments_filename = "experiments.csv"
commands = []
for a in agents:
    for r in rectangular_reasoning:
        for c in corridor_reasoning:
            for m in maps:
                for s in scens:
                    map_name = m.split('/')[1].split('.')
                    if (map_name[0] not in s):
                        # print(m, s)
                        continue
                    for d in decompose:
                        if d == "true":
                            for t in thresholds:
                                command = f"./cbs -m {m} -a {s} \
                                    -o {experiments_filename} \
                                    -k {a} -t 60 --decompose={d} --heuristics={r} \
                                    --corridorReasoning={c}\
                                    --threshold={t}"
                                commands.append(command)
                        else:
                            command = f"./cbs -m {m} -a {s} \
                                -o {experiments_filename} \
                                -k {a} -t 60 --decompose={d} --heuristics={r} \
                                --corridorReasoning={c}"
                            commands.append(command)
print(len(commands))


pool = multiprocessing.Pool()

pool.map(os.system, tqdm(commands))