import pandas as pd
data = pd.read_csv('experiments_boston_3.csv')
column_names = data.columns.tolist()
print(column_names)

grouped_rows = data.groupby(['number of agents', 'instance name'])
for name, group in grouped_rows:
    print(group['solver name'].shape)

unique_solver_names = data['solver name'].unique()
print(unique_solver_names)
unique_solver_completed = data.groupby('solver name')
# print(unique_solver_completed)
unique_num_agents = data['number of agents'].unique()
completed_count = {(num_agents, solver): 0 for num_agents, solver in zip(unique_num_agents, unique_solver_names)}
total_count = {(num_agents, solver): 0 for num_agents, solver in zip(unique_num_agents, unique_solver_names)}

for index, row in data.iterrows():
    solver = row['solver name']
    num_agents = row['number of agents']
    if row['solutionFound'] == 1:
        completed_count[(num_agents, solver)] = completed_count.get((num_agents, solver), 0) + 1
    total_count[(num_agents, solver)] = total_count.get((num_agents, solver), 0) + 1

import matplotlib.pyplot as plt

completion_rate = {(num_agents, solver): completed_count[(num_agents, solver)] / total_count[(num_agents, solver)] for (num_agents, solver) in completed_count}
algorithms = unique_solver_names
for algorithm in algorithms:
    if 'Decomp' not in algorithm:
        linestyle = '--'
    else:
        linestyle = '-'
    x = [num_agents for (num_agents, solver) in sorted(completion_rate) if solver == algorithm]
    y = [completion_rate[(num_agents, solver)] for (num_agents, solver) in sorted(completion_rate) if solver == algorithm]
    plt.plot(x, y, label=algorithm, linestyle=linestyle)
plt.xlabel('Number of Agents')
plt.ylabel('Completion Rate')
plt.title('Completion Rate by Algorithm and Number of Agents')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.show()
