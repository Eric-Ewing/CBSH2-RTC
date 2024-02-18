import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def make_completion_rate_per_map(data, map_name, sa_solver='Star'):
    # Filter out for our desired solver
    data = data[data['solver name'].str.contains('WDG\+GR\+GC\+T')]
    data = data[data['solver name'].str.contains(sa_solver)]

    # Rename solver, as it was misspelled in the cpp file
    if sa_solver == 'SIPP':
        sa_solver = 'SIPPS'

    # Set full name
    for i, row in data.iterrows():
        if 'Decomp' not in row['solver name']:
            row['fullSolverName'] = row['solver name']
        else:
            row['fullSolverName'] = row['solver name'] + ' ' + str(row['DecompThreshold'])
        data.at[i, 'fullSolverName'] = row['fullSolverName']

    unique_solver_names = data['fullSolverName'].unique()
    unique_num_agents = data['number of agents'].unique()

    completed_count = {(num_agents, solver): 0 for num_agents in unique_num_agents for  solver in unique_solver_names}
    total_count = {(num_agents, solver): 10 for num_agents in unique_num_agents for solver in unique_solver_names}

    for _, row in data.iterrows():
        solver = row['fullSolverName']
        num_agents = row['number of agents']
        if row['solutionFound'] == 1:
            if row['runtime'] + row['decompTime'] < 300: # Cutoff if decomp time caused issue
                completed_count[(num_agents, solver)] = completed_count.get((num_agents, solver), 0) + 1
            else:
                print('decomp caused timeout...')

    completion_rate = {(num_agents, solver): completed_count[(num_agents, solver)] / total_count[(num_agents, solver)] if total_count[(num_agents, solver)] !=0 else 0 for (num_agents, solver) in completed_count }
    algorithms = data['fullSolverName'].unique()

    ys = {a: [] for a in algorithms}
    xs = []

    for algorithm in algorithms:
        if 'Decomp' not in algorithm:
            linestyle = '--'
        else:
            linestyle = '-'
        x = [num_agents for (num_agents, solver) in sorted(completion_rate) if solver == algorithm and total_count[(num_agents, solver)] > 5]
        y = [completion_rate[(num_agents, solver)] for (num_agents, solver) in sorted(completion_rate) if solver == algorithm and total_count[(num_agents, solver)] > 5]
        
        # Delete any 0s from data (we'll readd the final one later)
        # When an algorithm fails, they may report an incorrect num agents for some reason
        for i in range(len(x) - 1, -1, -1):
            if y[i] == 0:
                del x[i]
                del y[i]

        # Make sure plot ends at 0.
        x.append(x[-1] + 5)
        y.append(0)

        if len(x) > len(xs):
            xs = x
        ys[algorithm] = y

        plt.plot(x, y, label=algorithm, linestyle=linestyle)

    plt.xlabel('Number of Agents')
    plt.ylabel('Completion Rate')
    plt.title('Completion Rate by Algorithm and Number of Agents')
    plt.legend(loc='lower left')
    plt.savefig('socs_results/CG+GR_{}_{}.pdf'.format(map_name, sa_solver))
    plt.clf()

def make_sorted_runtime(data, map_name):
    data = data[data['solver name'].str.contains('WDG\+GR\+GC\+T')]

    # Extract solver names
    # Set full name
    for i, row in data.iterrows():
        if 'Decomp' not in row['solver name']:
            row['fullSolverName'] = row['solver name']
        else:
            row['fullSolverName'] = row['solver name'] + ' ' + str(row['DecompThreshold'])
        data.at[i, 'fullSolverName'] = row['fullSolverName']

    unique_solver_names = data['fullSolverName'].unique()

    # Get all instances that were completed by solver
    for solver in unique_solver_names:
        solver_data = data[data['fullSolverName'] == solver]
        completed_instances = solver_data[solver_data['solutionFound'] == 1]
        runtimes = completed_instances['runtime'] + completed_instances['decompTime']

        sorted_runtimes = sorted(runtimes)

        if 'Decomp' not in solver:
            linestyle = '--'
        else:
            linestyle = '-'
        y = np.arange(len(sorted_runtimes))
        solver_label = solver.replace('WDG+GR+GC+T+BP', 'CBSH')
        solver_label = solver_label.replace(' with SIPP', '+SIPPS')
        solver_label = solver_label.replace(' with AStar', '+A*')
        solver_label = solver_label.replace(' and Decomp', '+θ=')
        plt.plot(sorted_runtimes, y, label=solver_label, linestyle=linestyle)

    # Plot
    plt.xscale('log')
    plt.xlabel('Time Limit (s)')
    plt.ylabel('Success Rate')
    plt.legend(fontsize=8)
    # plt.title(map_name)
    plt.savefig('socs_results/sorted_runtime_{}.pdf'.format(map_name))
    plt.clf()

# Make table for % of time spent on merging vs. Decomp vs. solving

# Make table for # Node expansions, % increase in f from component (+stdev?)

# Columns: Super columns for various maps (which maps?), columns for statistics
# Maps: HT_Mansion, Open
    
# rows for theta?
    

if __name__ == '__main__':
    all_data = pd.read_csv('experiments/socs_experiments_v6.csv')
    # all_data = pd.read_csv('bost_experiments.csv')
    # all_data = pd.read_csv('experiments_den312.csv')
    column_names = all_data.columns.tolist()
    # map_types = ['Boston', 'brc2', 'den312']
    map_types = ['maze', 'Berlin', 'brc', 'empty', 'ht_mansion', 'random']
    for m in map_types:
        data = all_data[all_data['instance name'].str.contains(m)]
        make_completion_rate_per_map(data, m, 'Star')
        make_completion_rate_per_map(data, m, 'SIPP')
        make_sorted_runtime(data, m)
