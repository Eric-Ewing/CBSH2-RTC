import pandas as pd
import numpy as np
from scipy import stats

def make_plots(data, map_name):
    # data = data[data['solver name'].str.contains('CG\+GR\+T')]
    # data = data[data['solver name'].str.contains('WDG\+GR\+T')]
    # data = data[data['solver name'].str.contains('CG\+GR\+GC\+T')]
    data = data[data['solver name'].str.contains('WDG\+GR\+GC\+T')]
    print(data)
    for i, row in data.iterrows():
        if 'Decomp' not in row['solver name']:
            row['fullSolverName'] = row['solver name']
        else:
            row['fullSolverName'] = row['solver name'][:-10] + ' ' + row['solver name'][-10:] + ' ' + str(row['DecompThreshold'])
        data.at[i, 'fullSolverName'] = row['fullSolverName']
    # data['fullSolverName'] = data['solver name'] + ' ' + data['DecompThreshold'].astype(str)
    unique_solver_names = data['fullSolverName'].unique()
    print(unique_solver_names)
    # print(unique_solver_completed)
    unique_num_agents = data['number of agents'].unique()
    completed_count = {(num_agents, solver): 0 for num_agents in unique_num_agents for  solver in unique_solver_names}
    total_count = {(num_agents, solver): 25 for num_agents in unique_num_agents for solver in unique_solver_names}
    print(completed_count)
    print(total_count)
    # Only did decomp once per instance and then stashed the files, so sometimes its 0 decomp time
    max_decomp_time = data.groupby(['number of agents', 'instance name', 'DecompThreshold'])['decompTime'].transform('max')
    data['decompTime'] = max_decomp_time / 12  # Divide by number of cores to get parallel comp time...
    for index, row in data.iterrows():
        solver = row['fullSolverName']
        num_agents = row['number of agents']
        if row['solutionFound'] == 1:
            if row['runtime'] + row['decompTime'] < 60: # Cutoff if decomp time caused issue
                completed_count[(num_agents, solver)] = completed_count.get((num_agents, solver), 0) + 1
        # total_count[(num_agents, solver)] = total_count.get((num_agents, solver), 0) + 1
    # print(stats.mode(completed_count.values()))
    import matplotlib.pyplot as plt

    completion_rate = {(num_agents, solver): completed_count[(num_agents, solver)] / total_count[(num_agents, solver)] if total_count[(num_agents, solver)] !=0 else 0 for (num_agents, solver) in completed_count }
    plt.clf()
    # print(total_count)
    algorithms = data['fullSolverName'].unique()
    ys = {a: [] for a in algorithms}
    xs = []
    for algorithm in algorithms:
        if 'Decomp' not in algorithm:
            linestyle = '--'
        else:
            linestyle = '-'
        # if total_count[(num_agents, solver)] < 5:
        #     continue
        x = [num_agents for (num_agents, solver) in sorted(completion_rate) if solver == algorithm and total_count[(num_agents, solver)] > 5]
        y = [completion_rate[(num_agents, solver)] for (num_agents, solver) in sorted(completion_rate) if solver == algorithm and total_count[(num_agents, solver)] > 5]
        for i in range(len(x) - 1, -1, -1):
            if y[i] == 0:
                del x[i]
                del y[i]
        x.append(x[-1] + 5)
        y.append(0)
        if len(x) > len(xs):
            xs = x
        ys[algorithm] = y
        if 'Decomp' in algorithm:
            y = [a / 2 for a in y]
        plt.plot(x, y, label=algorithm, linestyle=linestyle)
        print(x)
        print(y)
        print(algorithm)
    # max_decomp_ys = []
    # max_normal_ys = []
    # for a in algorithms:
    #     if 'Decomp' in a:
    #         for i, x in enumerate(ys[a]):
    #             if len(max_decomp_ys) <= i:
    #                 max_decomp_ys.append(x) 
    #             max_decomp_ys[i] = max(max_decomp_ys[i], x)
    #     else:
    #         print(a)
    #         for i, x in enumerate(ys[a]):
    #             if len(max_normal_ys) <= i:
    #                 max_normal_ys.append(x) 
    #             max_normal_ys[i] = max(max_normal_ys[i], x)
    # print(xs, max_decomp_ys)
    # plt.plot(xs, max_decomp_ys, label='Best Decomp. Config')
    # plt.plot(xs, max_normal_ys, label='Best CBSH Config.')
    plt.xlabel('Number of Agents')
    plt.ylabel('Completion Rate')
    plt.title('Completion Rate by Algorithm and Number of Agents')
    plt.legend(loc='lower left')
    plt.savefig('full_results/CG+GR_{}.pdf'.format(map_name))

if __name__ == '__main__':
    all_data = pd.read_csv('sipp_experiments.csv')
    # all_data = pd.read_csv('bost_experiments.csv')
    # all_data = pd.read_csv('experiments_den312.csv')
    column_names = all_data.columns.tolist()
    # map_types = ['Boston', 'brc2', 'den312']
    map_types = ['brc']
    for m in map_types:
        data = all_data[all_data['instance name'].str.contains(m)]
        make_plots(data, m)
        data = data[data['solutionFound'] == 1]
        data = data[data['number of agents'] == 50]
        # print(data['initialRuntime'].mean())
        # print(data['runtime'].mean())
        # print(data['decompTime']).mean()