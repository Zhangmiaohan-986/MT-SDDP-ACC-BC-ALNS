import os
import numpy as np
import random
import networkx as nx
import matplotlib.pyplot as plt
import pandas as pd
import math

def generate_points(num_points,seed):

    _here = os.path.dirname(os.path.abspath(__file__))
    file_path_R201 = os.path.join(_here, 'map_test', 'RC1_4_1.TXT')

    np.random.seed(seed)
    with open(file_path_R201, 'r') as file:
        data0_R201 = file.read()

    data_R201 = []
    for line in data0_R201.strip().split('\n'):
        data_R201.append(line.split())
    columns = ["CUST", "XCOORD.", 'YCOORD.', 'DEMAND', 'READY', 'DUE', 'SERVICE']
    df_R201 = pd.DataFrame(data_R201[1:], columns=columns)

    numeric_cols = ['CUST', 'XCOORD.', 'YCOORD.', 'DEMAND', 'READY', 'DUE', 'SERVICE']
    df_R201[numeric_cols] = df_R201[numeric_cols].apply(pd.to_numeric, errors='coerce')
    start_pos = (float(df_R201.at[0, 'XCOORD.']), float(df_R201.at[0, 'YCOORD.']))

    print("客户节点数量:", num_points)
    if num_points <= 500:

        depot_row = df_R201.iloc[[0]]

        customer_rows = df_R201.iloc[1:]

        if num_points <= 1:

            position_points_sample = depot_row.copy()
        else:
            sample_size = min(num_points - 1, len(customer_rows))
            sampled_customers = _sample_customers_diameter_and_min_neighbors(
                customer_rows=customer_rows,
                k=sample_size,
                seed=seed,
                max_pair_dist=100.0,
                near_dist=40.0,
                min_nb=2
            )

            position_points_sample = pd.concat([depot_row, sampled_customers], axis=0)

        position_points_sample = position_points_sample.sort_index()
        return position_points_sample, start_pos
    else:

        with open(file_path_RC101, 'r') as file:
            data0_RC101 = file.read()

        data_RC101 = []
        for line in data0_RC101.strip().split('\n'):
            data_RC101.append(line.split())

        df_RC101 = pd.DataFrame(data_RC101[1:], columns=columns)
        df_RC101[numeric_cols] = df_RC101[numeric_cols].apply(pd.to_numeric, errors='coerce')

        depot_row = df_R201.iloc[[0]]
        r201_customers = df_R201.iloc[1:]

        df_combine_no_depot = pd.concat(
            [r201_customers, df_RC101],
            ignore_index=True
        )

        if num_points <= 1:
            position_points_sample = depot_row.copy()
        else:
            sample_size = min(num_points - 1, len(df_combine_no_depot))
            sampled_customers = df_combine_no_depot.sample(
                n=sample_size,
                random_state=seed
            )
            position_points_sample = pd.concat(
                [depot_row, sampled_customers],
                axis=0
            )

        position_points_sample = position_points_sample.sort_index()
        return position_points_sample, start_pos

def generates_points(num_points, seed):
    _here = os.path.dirname(os.path.abspath(__file__))
    file_path_R1 = os.path.join(_here, 'map_test', 'R1_4_1.TXT')
    np.random.seed(seed)

    with open(file_path_R1, 'r') as file:
        data0_R1 = file.read()

    data_R1 = []
    for line in data0_R1.strip().split('\n'):
        data_R1.append(line.split())

    columns = ["CUST", "XCOORD.", 'YCOORD.', 'DEMAND', 'READY', 'DUE', 'SERVICE']
    df_R1 = pd.DataFrame(data_R1[1:], columns=columns)

    numeric_cols = ['CUST', 'XCOORD.', 'YCOORD.', 'DEMAND', 'READY', 'DUE', 'SERVICE']
    df_R1[numeric_cols] = df_R1[numeric_cols].apply(pd.to_numeric, errors='coerce')

    depot_row = df_R1.iloc[[0]]
    customer_rows = df_R1.iloc[1:]

    if num_points <= 1:

        position_points_sample = depot_row.copy()
    else:
        sample_size = min(num_points - 1, len(customer_rows))
        sampled_customers = customer_rows.sample(
            n=sample_size,
            random_state=seed
        )
        position_points_sample = pd.concat(
            [depot_row, sampled_customers],
            axis=0
        )

    position_points_sample = position_points_sample.sort_index()

    return position_points_sample

def generate_graph(position_points, seed, uav_distance=None, prob=0.75):
    if uav_distance is None:
        uav_distance = 50
    random.seed(seed)
    coordinates = list(zip(position_points['XCOORD.'], position_points['YCOORD.']))

    num_air = math.floor(len(coordinates) * 1 / 2)
    num_ground = int(len(coordinates) - num_air)

    coordinates_air = random.sample(coordinates, num_air)
    coordinates_air_matrix = compute_distance_matrix(coordinates_air)

    coordinates_ground = list(set(coordinates).difference(coordinates_air))
    coordinates_ground = list(coordinates_ground)
    coordinates_ground_matrix = compute_distance_matrix(coordinates_ground)

    G_air = nx.Graph()

    for i, coord in enumerate(coordinates_air):
        G_air.add_node(i, pos=coord)

    G_air = add_random_tree(G_air, coordinates_air_matrix, coordinates_air)

    G_air, air_positions, max_air_id, new_air_nodes = split_long_edges(G_air, uav_distance,
                                                                       starting_node_id=len(coordinates_air))

    G_ground = nx.Graph()
    for i, coord in enumerate(coordinates_ground):
        G_ground.add_node(i, pos=coord)

    G_ground = add_random_tree(G_ground, coordinates_ground_matrix, coordinates_ground, max_connect_num=4)

    ground_positions = nx.get_node_attributes(G_ground, 'pos')

    node_id_offset = max_air_id + 1
    mapping = {node: node + node_id_offset for node in G_ground.nodes()}
    G_ground = nx.relabel_nodes(G_ground, mapping)

    ground_positions = {node + node_id_offset: pos for node, pos in ground_positions.items()}

    air_adj_matrix = np.array(nx.adjacency_matrix(G_air).todense())
    ground_adj_matrix = np.array(nx.adjacency_matrix(G_ground).todense())

    return G_air, G_ground, air_adj_matrix, air_positions, ground_adj_matrix, ground_positions

def _sample_with_neighbor_bounds(customer_rows, k, seed,
                                 radius=40.0, min_nb=2, max_nb=3,
                                 trials=120):

    if k <= 0:
        return customer_rows.iloc[0:0].copy()
    if len(customer_rows) <= k:

        return customer_rows.copy()

    coords = customer_rows[['XCOORD.', 'YCOORD.']].to_numpy(dtype=float)
    n = len(coords)

    d = np.sqrt(((coords[:, None, :] - coords[None, :, :]) ** 2).sum(axis=2))
    np.fill_diagonal(d, np.inf)

    near = (d < radius)

    deg_full = near.sum(axis=1)
    feasible_mask = (deg_full >= min_nb)
    feasible_idx = np.where(feasible_mask)[0]
    if len(feasible_idx) < k:

        return customer_rows.iloc[feasible_idx].copy()

    rng = np.random.RandomState(seed)

    best_S = None
    best_score = -1

    for t in range(trials):

        start = int(rng.choice(feasible_idx))
        S = [start]
        inS = np.zeros(n, dtype=bool); inS[start] = True

        deg_in = np.zeros(n, dtype=int)

        while len(S) < k:
            candidates = np.where((~inS) & feasible_mask)[0]
            if len(candidates) == 0:
                break

            best_c = None
            best_c_score = -1e18

            need_more = (inS & (deg_in < min_nb))

            for c in candidates:
                neigh_to_S = near[c] & inS
                add_to = np.where(neigh_to_S)[0]

                if np.any(deg_in[add_to] + 1 > max_nb):
                    continue

                new_deg = int(neigh_to_S.sum())
                if new_deg > max_nb:
                    continue

                help_need = int((neigh_to_S & need_more).sum())
                score = (
                    3.0 * help_need +
                    0.4 * new_deg +
                    0.05 * deg_full[c]
                )

                score += rng.uniform(-0.15, 0.15)

                if score > best_c_score:
                    best_c_score = score
                    best_c = c

            if best_c is None:
                break

            inS[best_c] = True
            S.append(best_c)

            connected = np.where(near[best_c] & inS)[0]

            deg_in[connected] += 1
            deg_in[best_c] = int((near[best_c] & inS).sum())

        S_idx = np.array(S, dtype=int)
        degS = np.array([int((near[i] & inS).sum()) for i in S_idx], dtype=int)

        ok = np.sum((degS >= min_nb) & (degS <= max_nb))

        score_total = ok * 10 + len(S_idx)

        if score_total > best_score:
            best_score = score_total
            best_S = S_idx

        if len(S_idx) == k and ok == k:
            break

    if best_S is None:

        best_S = rng.choice(feasible_idx, size=k, replace=False)

    if len(best_S) > k:
        best_S = best_S[:k]

    return customer_rows.iloc[best_S].copy()

def _sample_customers_diameter_and_min_neighbors(
    customer_rows: pd.DataFrame,
    k: int,
    seed: int,
    max_pair_dist: float = 100.0,
    near_dist: float = 40.0,
    min_nb: int = 2,
    center_trials: int = 80,
    draw_trials: int = 120
):

    if k <= 0:
        return customer_rows.iloc[0:0].copy()
    if len(customer_rows) <= k:
        return customer_rows.copy()

    rng = np.random.RandomState(seed)
    coords = customer_rows[['XCOORD.', 'YCOORD.']].to_numpy(dtype=float)
    n = len(coords)

    D = np.sqrt(((coords[:, None, :] - coords[None, :, :]) ** 2).sum(axis=2))
    np.fill_diagonal(D, np.inf)

    R = max_pair_dist / 2.0

    center_ids = rng.choice(n, size=min(center_trials, n), replace=False)

    best_idx = None
    best_score = (-1, -1)

    for c in center_ids:
        center = coords[c]
        pool = np.where(np.linalg.norm(coords - center, axis=1) <= R + 1e-9)[0]
        if len(pool) < k:
            continue

        near_pool = (D[np.ix_(pool, pool)] <= near_dist + 1e-9)
        np.fill_diagonal(near_pool, False)
        deg_pool = near_pool.sum(axis=1)

        feasible_mask = (deg_pool >= min_nb)
        feasible_pool = pool[feasible_mask]
        if len(feasible_pool) < k:
            continue

        for t in range(draw_trials):
            idx = rng.choice(feasible_pool, size=k, replace=False)

            subD = D[np.ix_(idx, idx)]
            subNear = (subD <= near_dist + 1e-9)
            np.fill_diagonal(subNear, False)
            subDeg = subNear.sum(axis=1)

            satisfied = int(np.sum(subDeg >= min_nb))
            score = (satisfied, k)

            if score > best_score:
                best_score = score
                best_idx = idx

            if satisfied == k:

                return customer_rows.iloc[idx].copy()

    if best_idx is not None:
        return customer_rows.iloc[best_idx].copy()

    idx = rng.choice(n, size=k, replace=False)
    return customer_rows.iloc[idx].copy()

def split_long_edges(graph, threshold, starting_node_id):
    positions = nx.get_node_attributes(graph, 'pos')
    max_node_id = starting_node_id - 1
    edges_to_remove = []
    new_nodes = {}

    for u, v in list(graph.edges()):
        pos_u = positions[u]
        pos_v = positions[v]
        distance = np.linalg.norm(np.array(pos_u) - np.array(pos_v))
        if distance <= threshold:
            continue
        else:

            num_segments = int(np.ceil(distance / threshold))

            positions_list = []
            for i in range(1, num_segments):
                t = i / num_segments
                x_new = (1 - t) * pos_u[0] + t * pos_v[0]
                y_new = (1 - t) * pos_u[1] + t * pos_v[1]
                positions_list.append((x_new, y_new))

            new_node_ids = []
            for pos in positions_list:
                max_node_id += 1
                new_node_id = max_node_id
                positions[new_node_id] = pos
                graph.add_node(new_node_id, pos=pos)
                new_node_ids.append(new_node_id)
                new_nodes[new_node_id] = pos

            edges_to_remove.append((u, v))

            nodes_in_chain = [u] + new_node_ids + [v]
            for n1, n2 in zip(nodes_in_chain[:-1], nodes_in_chain[1:]):
                pos1 = positions[n1]
                pos2 = positions[n2]
                new_distance = np.linalg.norm(np.array(pos1) - np.array(pos2))
                graph.add_edge(n1, n2, weight=new_distance)

    graph.remove_edges_from(edges_to_remove)
    return graph, positions, max_node_id, new_nodes

def add_random_tree(G, distance_matrix, coordinates, max_connect_num=None):
    if max_connect_num is None:
        max_connect_num = 3
    num_nodes = len(distance_matrix)
    visited = np.full((num_nodes,num_nodes),False)
    coordinates_origin = (0,0,0)
    distance_origin = compute_distance_matrix_3D(coordinates, coordinates_origin)
    sorted_indices_origin = np.argsort(distance_origin)[0,0]
    near_path = find_nearest_path(distance_matrix,sorted_indices_origin)
    connect_count = {i: 0 for i in range(num_nodes)}
    for i in range(len(near_path)-1):
        current = near_path[i]
        next_current = near_path[i+1]
        current_prob = connect_prob(connect_count[current])
        if random.random() < current_prob:
            G.add_edge(current,next_current,weight=distance_matrix[current,next_current])
            connect_count[current] += 1
            connect_count[next_current] += 1
            visited[current,next_current] = True
            visited[next_current,current] = True

    for current in near_path:

        sorted_neighbors = np.argsort(distance_matrix[current])
        for neighbor in sorted_neighbors:

            if not visited[current, neighbor] and current != neighbor:

                if connect_count[current] < max_connect_num and connect_count[neighbor] < 10:

                    if random.random() < connect_prob(connect_count[current]):

                        G.add_edge(current, neighbor, weight=distance_matrix[current, neighbor])
                        visited[current, neighbor] = True
                        visited[neighbor, current] = True
                        connect_count[current] += 1
                        connect_count[neighbor] += 1
    return G

def find_nearest_path(distance_matrix, start_index):
    num_nodes = len(distance_matrix)
    visited = [False]*num_nodes
    path = []
    current_node = start_index
    while len(path)<num_nodes:
        visited[current_node] = True
        path.append(current_node)
        distance = distance_matrix[current_node]
        sorted_indices = np.argsort(distance)
        found_next = False
        for neighbor in sorted_indices:
            if not visited[neighbor] and neighbor != current_node:
                current_node = neighbor
                found_next = True
                break
        if not found_next:
            break
    return path

def connect_prob(connections):

    if connections == 0:
        return 1.0
    return max(0,1.0-0.2*connections)

def compute_distance_matrix(coordinates, origin=None):
    num_coordinates = len(coordinates)
    if origin is None:
        distance_matrix = np.zeros((num_coordinates, num_coordinates))
        for i in range(num_coordinates):
            for j in range(i + 1, num_coordinates):
                distance_matrix[i,j] = np.linalg.norm(np.array(coordinates[i]) - np.array(coordinates[j]))
                distance_matrix[j,i] = distance_matrix[i,j]
        return distance_matrix
    else:

        distance_matrix = np.linalg.norm(np.array(coordinates) - np.array(origin), axis=1).reshape(1, -1)
        return distance_matrix

def compute_distance_matrix_3D(coordinates, origin=None):

    coords_array = np.array(coordinates)

    if origin is None:
        num_coordinates = len(coordinates)
        distance_matrix = np.zeros((num_coordinates, num_coordinates))

        for i in range(num_coordinates):
            for j in range(i + 1, num_coordinates):

                distance_matrix[i, j] = np.linalg.norm(coords_array[i] - coords_array[j])
                distance_matrix[j, i] = distance_matrix[i, j]

        return distance_matrix
    else:

        origin_array = np.array(origin)
        distance_vector = np.linalg.norm(coords_array - origin_array, axis=1).reshape(1, -1)
        return distance_vector

def split_long_edges_with_type(graph, threshold, starting_node_id, edge_type):

    positions = nx.get_node_attributes(graph, 'pos')
    node_types = nx.get_node_attributes(graph, 'type')
    max_node_id = starting_node_id - 1
    edges_to_remove = []
    new_nodes = {}

    for u, v in list(graph.edges()):
        pos_u = np.array(positions[u])
        pos_v = np.array(positions[v])
        distance = np.linalg.norm(pos_u - pos_v)

        if distance <= threshold:
            continue
        else:

            num_segments = int(np.ceil(distance / threshold))

            positions_list = []
            for i in range(1, num_segments):
                t = i / num_segments
                new_pos = (1 - t) * pos_u + t * pos_v
                positions_list.append(tuple(new_pos))

            new_node_ids = []
            for pos in positions_list:
                max_node_id += 1
                new_node_id = max_node_id
                positions[new_node_id] = pos

                new_edge_type = edge_type
                graph.add_node(new_node_id, pos=pos, type=new_edge_type)
                new_node_ids.append(new_node_id)
                new_nodes[new_node_id] = pos

            edges_to_remove.append((u, v))

            nodes_in_chain = [u] + new_node_ids + [v]
            for n1, n2 in zip(nodes_in_chain[:-1], nodes_in_chain[1:]):
                pos1 = np.array(positions[n1])
                pos2 = np.array(positions[n2])
                new_distance = np.linalg.norm(pos1 - pos2)
                graph.add_edge(n1, n2, weight=new_distance)

    graph.remove_edges_from(edges_to_remove)

    return graph, positions, max_node_id, new_nodes

from mpl_toolkits.mplot3d import Axes3D
def visualize_tower_connections(G_air, G_ground):
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')

    G_air_pos = nx.get_node_attributes(G_air, 'pos')
    G_air_pos_3d = {node: (x, y, 45/1000) for node, (x, y) in G_air_pos.items()}

    air_x = [coord[0] for coord in G_air_pos_3d.values()]
    air_y = [coord[1] for coord in G_air_pos_3d.values()]
    air_z = [coord[2] for coord in G_air_pos_3d.values()]

    ax.scatter(air_x, air_y, air_z, c='red', label='Air Nodes')

    for edge in G_air.edges():
        node1, node2 = edge
        x_values = [G_air_pos_3d[node1][0], G_air_pos_3d[node2][0]]
        y_values = [G_air_pos_3d[node1][1], G_air_pos_3d[node2][1]]
        z_values = [G_air_pos_3d[node1][2], G_air_pos_3d[node2][2]]
        ax.plot(x_values, y_values, z_values, color='blue', linestyle='dashed', linewidth=2, alpha=0.6)

    G_ground_pos = nx.get_node_attributes(G_ground, 'pos')
    G_ground_pos_3d = {node: (x, y, 0) for node, (x, y) in G_ground_pos.items()}

    ground_x = [coord[0] for coord in G_ground_pos_3d.values()]
    ground_y = [coord[1] for coord in G_ground_pos_3d.values()]
    ground_z = [coord[2] for coord in G_ground_pos_3d.values()]

    ax.scatter(ground_x, ground_y, ground_z, c='b', marker='o', label='Ground Nodes')

    for edge in G_ground.edges():
        node1, node2 = edge
        x_values_ground = [G_ground_pos_3d[node1][0], G_ground_pos_3d[node2][0]]
        y_values_ground = [G_ground_pos_3d[node1][1], G_ground_pos_3d[node2][1]]
        z_values_ground = [G_ground_pos_3d[node1][2], G_ground_pos_3d[node2][2]]
        ax.plot(x_values_ground, y_values_ground, z_values_ground, color='yellow',  linewidth=2, alpha=0.6)

    ax.set_xlim(min(air_x) - 10, max(air_x) + 10)
    ax.set_ylim(min(air_y) - 10, max(air_y) + 10)
    ax.set_zlim(0, 25)

    ax.set_xlabel('X Coordinate', fontsize=12)
    ax.set_ylabel('Y Coordinate', fontsize=12)
    ax.set_zlabel('Z Coordinate', fontsize=12)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=12)

    plt.title("Benchmark Graph of a 50-Node Dual-Layer Road Network. test_points:6. generate_graph:1", fontsize=16)

    plt.show()

def generate_complex_network(num_points, seed, Z_coord, uav_distance, air_node_num, ground_node_num, customer_node_num):

    position_points, start_pos = generate_points(num_points, seed)

    time_windows_h = {}
    for _, row in position_points.iterrows():
        cust_id = int(row["CUST"])
        ready_h   = row["READY"]   / 60.0
        due_h     = row["DUE"]     / 60.0
        service_h = row["SERVICE"] / 60.0
        time_windows_h[cust_id] = {
            "ready_h":   ready_h,
            "due_h":     due_h,
            "service_h": service_h,
        }

    depot = position_points.iloc[0:1].copy()
    depot['Z_COORD'] = 0
    depot['NODE_TYPE'] = 'DEPOT'

    remaining_points = position_points.iloc[1:].copy()

    total_remaining = len(remaining_points)

    num_air_nodes = air_node_num
    num_vtp_nodes = ground_node_num
    num_customer_nodes = customer_node_num

    air_nodes = remaining_points.sample(n=num_air_nodes, random_state=seed)
    air_nodes['NODE_TYPE'] = 'AIR'
    air_nodes['Z_COORD'] = Z_coord

    remaining_after_air = remaining_points.drop(air_nodes.index)
    vtp_nodes = remaining_after_air.sample(n=num_vtp_nodes, random_state=seed+1)
    vtp_nodes['NODE_TYPE'] = 'VTP'
    vtp_nodes['Z_COORD'] = 0
    air_vtp_nodes = vtp_nodes.copy()
    air_vtp_nodes['NODE_TYPE'] = 'AIR'
    air_vtp_nodes['Z_COORD'] = Z_coord

    customer_nodes = remaining_after_air.drop(vtp_nodes.index)
    customer_nodes['NODE_TYPE'] = 'CUSTOMER'
    customer_nodes['Z_COORD'] = 0

    air_customer_nodes = customer_nodes.copy()
    air_customer_nodes['NODE_TYPE'] = 'AIR'
    air_customer_nodes['Z_COORD'] = Z_coord

    all_nodes = pd.concat([depot, air_nodes, vtp_nodes, customer_nodes, air_vtp_nodes, air_customer_nodes])

    return generate_complex_graph(all_nodes, depot, air_nodes, vtp_nodes, customer_nodes, air_vtp_nodes, air_customer_nodes, uav_distance, seed)

def generate_complex_graph(all_nodes, depot, air_nodes, vtp_nodes, customer_nodes, air_vtp_nodes, air_customer_nodes, uav_distance, seed):

    random.seed(seed)

    air_coordinates = list(zip(air_nodes['XCOORD.'], air_nodes['YCOORD.'], air_nodes['Z_COORD']))
    customer_coordinates = list(zip(customer_nodes['XCOORD.'], customer_nodes['YCOORD.'], customer_nodes['Z_COORD']))
    vtp_coordinates = list(zip(vtp_nodes['XCOORD.'], vtp_nodes['YCOORD.'], vtp_nodes['Z_COORD']))
    depot_coordinate = (depot['XCOORD.'].values[0], depot['YCOORD.'].values[0], depot['Z_COORD'].values[0])
    air_vtp_coordinates = list(zip(air_vtp_nodes['XCOORD.'], air_vtp_nodes['YCOORD.'], air_vtp_nodes['Z_COORD']))
    air_customer_coordinates = list(zip(air_customer_nodes['XCOORD.'], air_customer_nodes['YCOORD.'], air_customer_nodes['Z_COORD']))

    air_network_coords = air_coordinates + air_vtp_coordinates + air_customer_coordinates
    air_distance_matrix = compute_distance_matrix_3D(air_network_coords)
    G_air = nx.Graph()

    air_node_types = {}

    for i, coord in enumerate(air_coordinates):
        G_air.add_node(i, pos=coord, type='Aerial Relay Node')
        air_node_types[i] = 'Aerial Relay Node'

    vtp_offset = len(air_coordinates)

    for i, coord in enumerate(air_vtp_coordinates):
        node_id = i + vtp_offset
        G_air.add_node(node_id, pos=coord, type='VTP Takeoff/Landing Node')
        air_node_types[node_id] = 'VTP Takeoff/Landing Node'

    customer_offset = vtp_offset + len(air_vtp_coordinates)
    customer_positions = {}
    for i, coord in enumerate(air_customer_coordinates):
        node_id = i + customer_offset
        G_air.add_node(node_id, pos=coord, type='CUSTOMER Takeoff/Landing Node')
        air_node_types[node_id] = 'CUSTOMER Takeoff/Landing Node'
        customer_positions[node_id] = coord

    G_air = add_random_tree(G_air, air_distance_matrix, air_network_coords)
    air_node_type = 'Aerial Relay Node'

    G_air, air_positions, max_air_id, new_air_nodes = split_long_edges_with_type(
        G_air, uav_distance, starting_node_id=len(air_network_coords), edge_type=air_node_type
    )

    for new_node_id in new_air_nodes.keys():
        air_node_types[new_node_id] = air_node_type

    if new_air_nodes:

        all_air_positions = nx.get_node_attributes(G_air, 'pos')
        all_air_coords = list(all_air_positions.values())

        new_air_distance_matrix = compute_distance_matrix_3D(all_air_coords)

        G_air_temp = nx.Graph()
        for node_id, pos in all_air_positions.items():
            G_air_temp.add_node(node_id, pos=pos, type=air_node_types.get(node_id, air_node_type))

        for u, v in G_air.edges():
            G_air_temp.add_edge(u, v, weight=G_air[u][v].get('weight', 1.0))

        new_node_ids = list(new_air_nodes.keys())

        for node_id in new_node_ids:
            node_pos = all_air_positions[node_id]

            for other_id, other_pos in all_air_positions.items():

                if other_id == node_id or G_air_temp.has_edge(node_id, other_id):
                    continue

                dist = np.linalg.norm(np.array(node_pos) - np.array(other_pos))

                if dist <= uav_distance * 0.8 and random.random() < 0.45:
                    G_air_temp.add_edge(node_id, other_id, weight=dist)

        G_air = G_air_temp

    G_ground = nx.Graph()
    ground_node_types = {}

    G_ground.add_node(0, pos=depot_coordinate, type='DEPOT')
    ground_node_types[0] = 'DEPOT'

    for i, coord in enumerate(vtp_coordinates):
        node_id = i + 1
        G_ground.add_node(node_id, pos=coord, type='VTP')
        ground_node_types[node_id] = 'VTP'

    vtp_count = len(vtp_coordinates)
    for i, coord in enumerate(customer_coordinates):
        node_id = i + vtp_count + 1
        G_ground.add_node(node_id, pos=coord, type='CUSTOMER')
        ground_node_types[node_id] = 'CUSTOMER'

    ground_all_nodes = list(G_ground.nodes())
    for i in range(len(ground_all_nodes)):
        for j in range(i + 1, len(ground_all_nodes)):
            node_i = ground_all_nodes[i]
            node_j = ground_all_nodes[j]

            dist = np.linalg.norm(np.array(G_ground.nodes[node_i]['pos']) - np.array(G_ground.nodes[node_j]['pos']))

            G_ground.add_edge(node_i, node_j, weight=dist)
    air_positions = nx.get_node_attributes(G_air, 'pos')
    ground_positions = nx.get_node_attributes(G_ground, 'pos')

    air_adj_matrix = np.array(nx.adjacency_matrix(G_air).todense())
    ground_adj_matrix = np.array(nx.adjacency_matrix(G_ground).todense())

    original_format_data = all_nodes.copy()

    relay_data = []
    for node_id, pos in new_air_nodes.items():
        relay_row = {
            'CUST': f'RELAY_{node_id}',
            'XCOORD.': pos[0],
            'YCOORD.': pos[1],
            'Z_COORD': pos[2],
            'DEMAND': 0,
            'READY': 0,
            'DUE': 9999,
            'SERVICE': 0,
            'NODE_TYPE': 'Aerial Relay Node New'
        }
        relay_data.append(relay_row)

    if relay_data:
        relay_df = pd.DataFrame(relay_data)
        original_format_data = pd.concat([original_format_data, relay_df])

    customer_time_windows_h = {}

    for node_id, coord in customer_positions.items():
        x, y, z = coord

        mask = (
            np.isclose(original_format_data['XCOORD.'], x) &
            np.isclose(original_format_data['YCOORD.'], y) &
            (original_format_data['NODE_TYPE'] == 'CUSTOMER')
        )
        matched = original_format_data[mask]

        if matched.empty:

            continue

        row = matched.iloc[0]
        cust_id   = int(row['CUST'])
        ready_h   = row['READY']   / 60.0
        due_h     = row['DUE']     / 60.0
        service_h = row['SERVICE'] / 60.0

        customer_time_windows_h[node_id] = {
            "cust":      cust_id,
            "ready_h":   ready_h,
            "due_h":     due_h,
            "service_h": service_h,
        }

    return (
        G_air,
        G_ground,
        air_adj_matrix,
        air_positions,
        ground_adj_matrix,
        ground_positions,
        original_format_data,
        air_node_types,
        ground_node_types,
        customer_time_windows_h
    )

def visualize_tower_connections_3D(G_air, G_ground, all_data, air_node_types, ground_node_types, fig_size=(10, 8), dpi=150, save_path=None):

    plt.rcParams.update({
        'font.family': 'Arial',
        'font.size': 12,
        'axes.linewidth': 1.5,
        'axes.labelsize': 14,
        'axes.titlesize': 16,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'legend.fontsize': 10,
        'legend.frameon': True,
        'legend.framealpha': 0.7,
        'legend.edgecolor': 'k'
    })

    fig = plt.figure(figsize=fig_size, dpi=dpi)
    ax = fig.add_subplot(111, projection='3d')

    air_positions = nx.get_node_attributes(G_air, 'pos')
    ground_positions = nx.get_node_attributes(G_ground, 'pos')

    node_colors = {
        'DEPOT': '#E63946',
        'VTP': '#457B9D',
        'CUSTOMER': '#2A9D8F',
        'Aerial Relay Node': '#F3722C',
        'VTP Takeoff/Landing Node': '#8338EC',
        'CUSTOMER Takeoff/Landing Node': '#FF66CC',
        'Aerial Relay Node New': '#FFD60A'
    }

    node_markers = {
        'DEPOT': '*',
        'VTP': 's',
        'CUSTOMER': 'o',
        'Aerial Relay Node': '^',
        'VTP Takeoff/Landing Node': 'D',
        'CUSTOMER Takeoff/Landing Node': 'h',
        'Aerial Relay Node New': 'X'
    }

    node_sizes = {
        'DEPOT': 150,
        'VTP': 80,
        'CUSTOMER': 60,
        'Aerial Relay Node': 80,
        'VTP Takeoff/Landing Node': 80,
        'CUSTOMER Takeoff/Landing Node': 60,
        'Aerial Relay Node New': 50
    }

    node_to_data = {}
    for _, row in all_data.iterrows():
        if 'CUST' in row and 'NODE_TYPE' in row:
            node_to_data[row['CUST']] = row

    for _, row in all_data.iterrows():
        if 'CUST' in row and str(row['CUST']).startswith('RELAY_'):
            try:
                node_id = int(str(row['CUST']).replace('RELAY_', ''))
                node_to_data[node_id] = row
            except:
                continue

    all_node_types = {}
    all_node_types.update(air_node_types)
    all_node_types.update(ground_node_types)

    plotted_types = set()

    ax.set_facecolor('#f8f9fa')

    for node in G_air.nodes():
        node_type = air_node_types.get(node, 'Aerial Relay Node')
        pos = air_positions[node]

        if node_type not in plotted_types:
            scatter = ax.scatter(
                pos[0], pos[1], pos[2],
                s=node_sizes.get(node_type, 60),
                c=node_colors.get(node_type, 'gray'),
                marker=node_markers.get(node_type, 'o'),
                edgecolors='black',
                linewidths=0.8,
                alpha=0.9,
                label=node_type,
                zorder=10
            )
            plotted_types.add(node_type)
        else:
            scatter = ax.scatter(
                pos[0], pos[1], pos[2],
                s=node_sizes.get(node_type, 60),
                c=node_colors.get(node_type, 'gray'),
                marker=node_markers.get(node_type, 'o'),
                edgecolors='black',
                linewidths=0.8,
                alpha=0.9,
                zorder=10
            )

    for edge in G_air.edges():
        node1, node2 = edge
        if node1 in air_positions and node2 in air_positions:
            pos1 = air_positions[node1]
            pos2 = air_positions[node2]

            ax.plot(
                [pos1[0], pos2[0]],
                [pos1[1], pos2[1]],
                [pos1[2], pos2[2]],
                color='#666666',
                linestyle=(0, (3, 3)),
                linewidth=1.5,
                alpha=0.6,
                zorder=5
            )

    for node in G_ground.nodes():
        node_type = ground_node_types.get(node, 'UNKNOWN')
        pos = ground_positions[node]

        if node_type not in plotted_types:
            ax.scatter(
                pos[0], pos[1], pos[2],
                s=node_sizes.get(node_type, 60),
                c=node_colors.get(node_type, 'gray'),
                marker=node_markers.get(node_type, 'o'),
                edgecolors='black',
                linewidths=0.8,
                alpha=0.9,
                label=node_type,
                zorder=10
            )
            plotted_types.add(node_type)
        else:
            ax.scatter(
                pos[0], pos[1], pos[2],
                s=node_sizes.get(node_type, 60),
                c=node_colors.get(node_type, 'gray'),
                marker=node_markers.get(node_type, 'o'),
                edgecolors='black',
                linewidths=0.8,
                alpha=0.9,
                zorder=10
            )

    vtp_ground_nodes = [n for n in G_ground.nodes() if ground_node_types.get(n) == 'VTP']
    for g_node in vtp_ground_nodes:
        g_pos = ground_positions[g_node]

        matching_air_nodes = []
        for a_node in G_air.nodes():
            if air_node_types.get(a_node) == 'VTP Takeoff/Landing Node':
                a_pos = air_positions[a_node]
                if abs(a_pos[0] - g_pos[0]) < 0.01 and abs(a_pos[1] - g_pos[1]) < 0.01:
                    matching_air_nodes.append(a_node)

        for a_node in matching_air_nodes:
            a_pos = air_positions[a_node]
            ax.plot(
                [g_pos[0], a_pos[0]],
                [g_pos[1], a_pos[1]],
                [g_pos[2], a_pos[2]],
                color='#457B9D',
                linestyle=(0, (2, 2)),
                linewidth=1.2,
                alpha=0.7,
                zorder=1
            )

    customer_ground_nodes = [n for n in G_ground.nodes() if ground_node_types.get(n) == 'CUSTOMER']
    for g_node in customer_ground_nodes:
        g_pos = ground_positions[g_node]

        matching_air_nodes = []
        for a_node in G_air.nodes():
            if air_node_types.get(a_node) == 'CUSTOMER Takeoff/Landing Node':
                a_pos = air_positions[a_node]
                if abs(a_pos[0] - g_pos[0]) < 0.01 and abs(a_pos[1] - g_pos[1]) < 0.01:
                    matching_air_nodes.append(a_node)

        for a_node in matching_air_nodes:
            a_pos = air_positions[a_node]
            ax.plot(
                [g_pos[0], a_pos[0]],
                [g_pos[1], a_pos[1]],
                [g_pos[2], a_pos[2]],
                color='#2A9D8F',
                linestyle=(0, (2, 2)),
                linewidth=1.2,
                alpha=0.7,
                zorder=1
            )

    x_values = [pos[0] for pos in air_positions.values()] + [pos[0] for pos in ground_positions.values()]
    y_values = [pos[1] for pos in air_positions.values()] + [pos[1] for pos in ground_positions.values()]
    z_values = [pos[2] for pos in air_positions.values()] + [pos[2] for pos in ground_positions.values()]

    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    z_min, z_max = min(z_values), max(z_values)

    x_margin = (x_max - x_min) * 0.1
    y_margin = (y_max - y_min) * 0.1
    z_margin = max((z_max - z_min) * 0.1, 1)

    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)
    ax.set_zlim(z_min, z_max + z_margin * 2)

    ax.set_xlabel('X (Operation Range, km)', fontweight='bold', labelpad=10, fontsize=12)
    ax.set_ylabel('Y (Operation Range, km)', fontweight='bold', labelpad=10, fontsize=12)
    ax.set_zlabel('Z (Altitude, m)', fontweight='bold', labelpad=10, fontsize=12)

    ax.grid(True, linestyle='--', alpha=0.4, linewidth=0.5)

    ax.xaxis.set_major_formatter(plt.FormatStrFormatter('%.0f'))
    ax.yaxis.set_major_formatter(plt.FormatStrFormatter('%.0f'))
    ax.zaxis.set_major_formatter(plt.FormatStrFormatter('%.0f'))

    ax.xaxis.set_major_locator(plt.MaxNLocator(5))
    ax.yaxis.set_major_locator(plt.MaxNLocator(5))
    ax.zaxis.set_major_locator(plt.MaxNLocator(4))

    from matplotlib.lines import Line2D

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))

    custom_lines = [
        Line2D([0], [0], color='#666666', lw=1.5, linestyle=(0, (3, 3))),
        Line2D([0], [0], color='#457B9D', lw=1.5, linestyle=(0, (2, 2))),
        Line2D([0], [0], color='#2A9D8F', lw=1.5, linestyle=(0, (2, 2)))
    ]

    all_handles = list(by_label.values()) + custom_lines
    all_labels = list(by_label.keys()) + [
        'Urban Aerial Corridor',
        'VTP Vertical Takeoff/Landing Corridor',
        'Customers Vertical Takeoff/Landing Corridor'
    ]

    legend = ax.legend(
        all_handles, all_labels,
        loc='upper right',
        fontsize=8,
        ncol=2,
        bbox_to_anchor=(1.0, 1.0),
        frameon=True,
        framealpha=0.8,
        edgecolor='black',
        markerscale=0.8,
        handlelength=1.5,
        handletextpad=0.5,
        columnspacing=1.0,
        borderpad=0.6,
        title='Network Elements',
        title_fontsize=10
    )
    legend.get_frame().set_linewidth(0.8)

    plt.title('Vehicle-UAV Delivery Environment', fontweight='bold', pad=20, fontsize=14)

    ax.view_init(elev=30, azim=45)

    plt.tight_layout()
    _here = os.path.dirname(os.path.abspath(__file__))
    save_path = os.path.join(_here, 'map_test.png')

    if save_path:
        plt.savefig(save_path, dpi=dpi, bbox_inches='tight', pad_inches=0.1)

    plt.show()

    return fig

if __name__ == '__main__':

    num_points = 50
    seed = 6
    Z_coord = 5
    uav_distance = 15
    G_air, G_ground, air_adj_matrix, air_positions, ground_adj_matrix, ground_positions, all_data, air_node_types, ground_node_types = generate_complex_network(num_points, seed, Z_coord, uav_distance)

    visualize_tower_connections_3D(G_air, G_ground, all_data, air_node_types, ground_node_types)
