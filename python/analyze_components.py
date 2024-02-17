# Average size of largest component / num_agents by num_agents and threshold
import glob
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams.update({'font.size': 16})
def box_per_theta(theta):
    all_data = {}
    for component_file in glob.glob('../components/scens/brc2*'):
        with open(component_file, 'r') as f:
            threshold = float('0.'+component_file.split('.scen')[1].split('0.')[1])
            if threshold != theta:
                continue
            lines = f.readlines()
            num_agents_per_component = []
            for component in lines:
                agents = component.split(',')
                num_agents_per_component.append(len(agents) - 1)
            if len(num_agents_per_component) == 0:
                num_agents_per_component.append(1)
            total_num_agents = int(component_file.split('.scen')[1].split('0.')[0])
            largest_component = max(num_agents_per_component)
            if total_num_agents in all_data:
                all_data[total_num_agents].append(largest_component/total_num_agents)
            else:
                all_data[total_num_agents] = [largest_component/total_num_agents]
    sorted_agents = sorted(list(all_data.keys()))
    while(sorted_agents[-1] > 105):
        sorted_agents.pop()
    plot_data = np.array([np.mean(all_data[a]) for a in sorted_agents])
    plot_std = np.array([np.std(all_data[a]) for a in sorted_agents])
    # plt.violinplot(plot_data, sorted_agents)
    plt.tight_layout()
    plt.ylim(0, 1.0)
    if theta == 0:
        plt.ylabel('Max Component Ratio')
    plt.xlabel('# Agents')
    plt.plot(sorted_agents, plot_data, label=theta)
    plt.fill_between(sorted_agents, plot_data+plot_std, plot_data-plot_std, alpha=0.25)
    # plt.savefig('threshold_{}.pdf'.format(theta), bbox_inches='tight')
    plt.legend(loc='lower right')
    # plt.clf()




box_per_theta(0)
box_per_theta(0.1)
box_per_theta(0.2)

all_data = pd.read_csv('../experiments_boston_3.csv')
data = all_data[all_data['instance name'].str.contains('Boston')]
filtered_data = data.loc[data['number of agents'] % 5 == 0]
filtered_data['decompTime'] = filtered_data.groupby(['number of agents', 'instance name', 'DecompThreshold'])['decompTime'].transform('max') / 20
filtered_data = filtered_data[filtered_data['number of agents'] < 100]
avg_decomp_time = filtered_data.groupby('number of agents')['decompTime'].mean()
std_decomp_time = filtered_data.groupby('number of agents')['decompTime'].std()
# avg_decomp_time['decompTime'] /= 12
ax1 = plt.gca()
ax2 = ax1.twinx()

ax2.plot(avg_decomp_time, label='decomp. time', linestyle='--', color='red')
ax2.fill_between(range(5, 100, 5), std_decomp_time+avg_decomp_time, avg_decomp_time - std_decomp_time, color='red', alpha=0.25)
ax2.tick_params(axis='y', labelcolor='red')
ax2.set_ylabel('Decomp. Time (s)', color='red')
ax2.legend(loc='lower center')
plt.savefig('thresholds.pdf', bbox_inches='tight')