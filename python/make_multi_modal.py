import glob
import numpy as np
import random
import matplotlib.pyplot as plt
import copy

ver = 14
def save_scen(a1, g1, a2, g2, name):
    a1.extend(a2)
    g1.extend(g2)
    l = list(zip(a1, g1))
    random.shuffle(l)
    a1, g1 = zip(*l)
    output = "version 1\n"
    for i, (a, g) in enumerate(zip(a1, g1)):
        output += f"{i}\t{name}\t{1}\t{1}\t{a[0]}\t{a[1]}\t{g[0]}\t{g[1]}\t1\n"
    prefix = name.split('.')[-2].split('/')[-1]
    output_path = f'../scens/bimodal_scens/{prefix}-bimodal-{ver}.scen'
    with open(output_path, 'w') as f:
        f.write(output)
while True:
    for filename in glob.glob('../socs_maps/*.map'):
        if 'empty' not in filename:
            continue
        with open(filename, 'r') as f:
            map = f.read()
        free_space = []
        edited_map = map.split('\n')[4:]
        for i, l in enumerate(edited_map):
            for j, c in enumerate(l):
                if c == '.':
                    free_space.append((i, j))
        start_mean = random.choice(free_space)
        goals_mean = random.choice(free_space)
        while(np.linalg.norm(np.array(start_mean) - np.array(goals_mean)) < 20):
            goals_mean = random.choice(free_space)
        free = free_space
        var = 10
        p_start = np.array([np.exp(-((c[0] - start_mean[0])**2/(2*var**2) + (c[1] - start_mean[1])**2 / (2 * var**2))) for c in free])
        p_start /= p_start.sum()
        p_goals = np.array([np.exp(-((c[0] - goals_mean[0])**2/(2*var**2) + (c[1] - goals_mean[1])**2 / (2 * var**2))) for c in free])
        p_goals /= p_goals.sum()


        agents_idx = np.random.choice(range(len(free)), 200, replace=False, p=p_start)
        goals_idx = np.random.choice(range(len(free)), 200, replace=False, p=p_goals)

        agents = [free[a] for a in agents_idx]
        goals = [free[g] for g in goals_idx]

        swap = np.random.randint(0, len(agents))
        temp = copy.copy(agents)
        agents[:swap] = goals[:swap]
        goals[:swap] = temp[:swap]

        map = np.zeros((max([f[0] for f in free]) + 1, max([f[1] for f in free]) + 1))
        start_mean2 = random.choice(free_space)
        goals_mean2 = random.choice(free_space)
        while(np.linalg.norm(np.array(start_mean2) - np.array(goals_mean2)) < 20):
            goals_mean2 = random.choice(free_space)
        p_start = np.array([np.exp(-((c[0] - start_mean2[0])**2/(2*var**2) + (c[1] - start_mean2[1])**2 / (2 * var**2))) for c in free])
        p_start /= p_start.sum()
        p_goals = np.array([np.exp(-((c[0] - goals_mean2[0])**2/(2*var**2) + (c[1] - goals_mean2[1])**2 / (2 * var**2))) for c in free])
        p_goals /= p_goals.sum()
        agents_idx_2 = np.random.choice(range(len(free)), 200, replace=False, p=p_start)
        goals_idx_2 = np.random.choice(range(len(free)), 200, replace=False, p=p_goals)

        agents_2 = [free[a] for a in agents_idx_2]
        goals_2 = [free[g] for g in goals_idx_2]
        temp = []
        for a in agents_2:
            if a not in agents and a not in goals:
                temp.append(a)
        agents_2 = temp

        temp = []
        for g in goals_2:
            if g not in goals and g not in agents:
                temp.append(g)
        g = temp

        agents_2, goals_2 = zip(*zip(agents_2, goals_2))
        agents_2 = list(agents_2)
        goals_2 = list(goals_2)
        swap = np.random.randint(0, len(agents_2))
        temp = copy.copy(agents_2)
        agents_2[:swap] = goals_2[:swap]
        goals_2[:swap] = temp[:swap]
        # map = np.zeros((max([f[0] for f in free]) + 1, max([f[1] for f in free]) + 1))
        print(agents_2)
        print(agents)
        for f in free_space:
            map[f[0], f[1]] = 1
        for a in agents:
            map[a[0], a[1]] = 7.5
        
        for g in goals:
            map[g[0], g[1]] = 5
        
        for a in agents_2:
            map[a[0], a[1]] = 10
        
        for g in goals_2:
            map[g[0], g[1]] = 15.5

        
        plt.imshow(map)
        plt.plot((start_mean2[1], goals_mean2[1]), (start_mean2[0], goals_mean2[0]))
        plt.plot((start_mean[1], goals_mean[1]), (start_mean[0], goals_mean[0]))
        plt.show()
        # plt.show()
        save = input()
        if save == 'y':
            save_scen(agents, goals, agents_2, goals_2, filename)
            ver += 1
