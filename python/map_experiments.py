import numpy as np
import random
import networkx as nx
import pickle
import matplotlib.pyplot as plt

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

with open('../maps/empty_210.map', 'r') as f:
    map = f.read()

free_space = []
edited_map = map.split('\n')[4:]
for i, l in enumerate(edited_map):
    # print(l)
    for j, c in enumerate(l):
        if c == '.' and i < 210 and j < 210:
            free_space.append((j, i))
            # print(free_space)

def betweenness_cent():

    G = make_graph(free_space)
    with open('brc202_graph.pkl', 'wb') as f:
        pickle.dump(G, f)

    centrality = nx.centrality.betweenness_centrality(G, k=1000)
    # print(centrality)
    map = np.zeros((210, 210))
    for v, bc in centrality.items():
        map[v[0], v[1]] = bc
    plt.imshow(map, cmap='hot')
    plt.show()

def make_unif_scens():
    # print(free_space)
    for i in range(2000):
        starts = random.sample(free_space, 500)
        goals = random.sample(free_space, 500)

        output_file = 'version 1 \n'

        for j, (s, g) in enumerate(zip(starts, goals)):
            line = '{}\tempty_210.map\t210\t210\t{}\t{}\t{}\t{}\t99999\n'.format(j, s[0], s[1], g[0], g[1])
            output_file += line

        with open('unif_scen/empty_210_{}.scen'.format(i), 'w') as f:
            f.write(output_file)

betweenness_cent()
# make_unif_scens()