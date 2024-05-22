from glob import glob
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable
import networkx as nx

def uniform():
     count_dict = {}
     # height = 481
     # width = 530
     height = 210
     width=210
     for conflicts in glob('../empty_uniform_experiments/*.tree'):
          with open(conflicts, 'r') as f:
               all_conflicts = f.read()
               agent_vertex_pairs = set()
               for c in all_conflicts.split('\n'):
                    if len(c) <= 1:
                         break
                    type = c.split(',')[-1][0]
                    if type == 'V':
                         agent = c.split(',')[0].split('<')[1]
                         vertex = c.split(',')[1]
                         if (agent, vertex) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent, vertex))
                              vertex = (int(vertex) % width, int(vertex) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1
                    elif type == 'E':
                         agent1 = c.split(',')[0].split('<')[1]
                         vertex1 = c.split(',')[1]
                         vertex2 = c.split(',')[2]
                         if (agent1, vertex1) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent1, vertex1))
                              vertex = (int(vertex1) % width, int(vertex1) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1
                         if (agent1, vertex2) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent1, vertex2))
                              vertex = (int(vertex2) % width, int(vertex2) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1


     grid = np.zeros((width, height))
     for v, count in count_dict.items():
          grid[v[0], v[1]] = count
     print('empty avg BC: ', np.mean([v for v in count_dict.values()]) / len(count_dict.values()))
     # grid /= np.sum(grid)
     # print(np.sum(grid))

     # print(np.histogram(grid, bins=100))
     # Run the same experiment with open map w/ same # nodes
     cmap = plt.cm.YlOrRd
     cmap.bad = 'black'

     im = plt.imshow(grid, vmax=100, cmap=cmap)
     plt.colorbar()
     plt.tight_layout()
     plt.savefig('bc_conflicts_figures/empty_conflict_count.pdf')
     # print(np.histogram(grid, bins=100))
     plt.clf()
     plt.hist(grid.flatten())
     plt.yscale('log')
     plt.ylabel('# Occurances')
     plt.xlabel('# Conflicts')
     plt.tight_layout()
     plt.savefig('bc_conflicts_figures/empty_hist_conflicts.pdf')
     plt.clf()

def brc202d():
     count_dict = {}
     height = 481
     width = 530
     for conflicts in glob('../brc_uniform_experiments/*.tree'):
          with open(conflicts, 'r') as f:
               all_conflicts = f.read()
               agent_vertex_pairs = set()
               for c in all_conflicts.split('\n'):
                    if len(c) <= 1:
                         break
                    type = c.split(',')[-1][0]
                    if type == 'V':
                         agent = c.split(',')[0].split('<')[1]
                         vertex = c.split(',')[1]
                         if (agent, vertex) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent, vertex))
                              vertex = (int(vertex) % width, int(vertex) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1
                    elif type == 'E':
                         agent1 = c.split(',')[0].split('<')[1]
                         vertex1 = c.split(',')[1]
                         vertex2 = c.split(',')[2]
                         if (agent1, vertex1) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent1, vertex1))
                              vertex = (int(vertex1) % width, int(vertex1) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1
                         if (agent1, vertex2) not in agent_vertex_pairs:
                              agent_vertex_pairs.add((agent1, vertex2))
                              vertex = (int(vertex2) % width, int(vertex2) // width)
                              if vertex not in count_dict:
                                   count_dict[vertex] = 0
                              count_dict[vertex] += 1


     grid = np.zeros((width, height))
     for v, count in count_dict.items():
          grid[v[0], v[1]] = count
     # grid /= np.sum(grid)
     # print(np.sum(grid))
     print('brc202d avg BC: ', np.mean([v for v in count_dict.values()]) / len(count_dict.values()))
     cmap = plt.cm.YlOrRd
     cmap.bad = 'black'
     global brc2_mask
     grid = np.ma.masked_where(brc202d_mask == 0, grid)
     plt.imshow(grid, vmax=1000, cmap=cmap)
     plt.colorbar()
     plt.imshow(grid, vmax=50, cmap=cmap)
     plt.savefig('bc_conflicts_figures/brc202d_conflict_count.pdf')
     plt.clf()
     # print(np.histogram(grid, bins=100))
     plt.hist(grid.flatten(), bins=np.linspace(1, 4000, 100))
     plt.yscale('log')
     plt.ylabel('# Occurances')
     plt.xlabel('# Conflicts')
     plt.tight_layout()
     plt.savefig('bc_conflicts_figures/brc202d_hist_conflicts.pdf')
     plt.clf()
     # Run the same experiment with open

def make_graph(free_space):
    G = nx.Graph()
    for i, (x, y) in enumerate(free_space):
        G.add_node((x, y), pos=(x, y))
    for node in G.nodes():
        if (node[0] + 1, node[1]) in G.nodes():
            G.add_edge(node, (node[0] + 1, node[1]))
        if (node[0] - 1, node[1]) in G.nodes():
            G.add_edge(node, (node[0] - 1, node[1]))
        if (node[0], node[1] + 1) in G.nodes():
            G.add_edge(node, (node[0], node[1] + 1))
        if (node[0], node[1] - 1) in G.nodes():
            G.add_edge(node, (node[0], node[1] - 1))
    return G

def betweenness_centrality_empty():

     with open('../maps/empty_210.map', 'r') as f:
          map = f.read()

     free_space = []
     edited_map = map.split('\n')[4:]
     for i, l in enumerate(edited_map):
          for j, c in enumerate(l):
               if c == '.' and i < 210 and j < 210:
                    free_space.append((j, i))
                    # print(free_space)


     G = make_graph(free_space)

     centrality = nx.centrality.betweenness_centrality(G, k=2000)
     # print(centrality)
     map = np.zeros((210, 210))
     for v, bc in centrality.items():
          map[v[0], v[1]] = bc
     # map = np.ma.masked_where(map != 0, map)
     plt.imshow(map, cmap='YlOrRd')
     plt.colorbar()
     plt.savefig('bc_conflicts_figures/empty_betweenness_centrality.pdf')
     plt.clf()

     plt.hist(centrality.values(), bins=10)
     plt.yscale('log')
     plt.ylabel('# Occurances')
     plt.xlabel('Betweenness Centrality')
     plt.savefig('bc_conflicts_figures/empty_hist_betweenness_centrality.pdf')
     plt.clf()

def betweenness_centrality_brc():
     height = 481
     width = 530
     with open('../maps/brc202d.map', 'r') as f:
          map = f.read()

     free_space = []
     edited_map = map.split('\n')[4:]
     for i, l in enumerate(edited_map):
          for j, c in enumerate(l):
               if c == '.' and i < width and j < height:
                    free_space.append((j, i))
                    # print(free_space)


     G = make_graph(free_space)

     centrality = nx.centrality.betweenness_centrality(G, k=2000)
     # print(centrality)
     map = np.zeros((width, height))
     for v, bc in centrality.items():
          map[v[0], v[1]] = bc
     mask = np.zeros((width, height))
     for v in G.nodes:
          mask[v[0], v[1]] = 1
     global brc202d_mask
     brc202d_mask = mask
     map = np.ma.masked_where(mask == 0, map)
     cmap = plt.cm.YlOrRd
     cmap.set_bad('black')
     plt.imshow(map, vmax=0.25, cmap=cmap)
     plt.colorbar()
     plt.savefig('bc_conflicts_figures/brc202d_betweenness_centrality.pdf')
     plt.clf()

     plt.hist(centrality.values(), bins=100)
     plt.yscale('log')
     plt.ylabel('# Occurances')
     plt.xlabel('Betweenness Centrality')
     plt.savefig('bc_conflicts_figures/brc202d_hist_betweenness_centrality.pdf')
     plt.clf()


# Make a (2, 2) grid of plots
# fig, axes = plt.subplots(3, 2, figsize=(10, 10))

betweenness_centrality_brc()
betweenness_centrality_empty()
brc202d()
uniform()

