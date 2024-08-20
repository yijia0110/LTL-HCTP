from collections import defaultdict

# 构建邻接表
def build_graph(edges):
    graph = defaultdict(list)
    for start, end in edges:
        graph[start].append(end)
    return graph

# 使用迭代方法查找所有路径
def find_all_paths_iterative(graph, start, end):
    stack = [(start, [start])]  # 栈中保存当前节点和路径
    paths = []
    
    while stack:
        vertex, path = stack.pop()
        
        for next_node in graph[vertex]:
            if next_node not in path:  # 防止环
                new_path = path + [next_node]
                if next_node == end:
                    paths.append(new_path)
                else:
                    stack.append((next_node, new_path))
                    
    return paths

# 边列表
edges = [
    ('T3_init', 'T3_S2'), ('T3_init', 'T3_init'), ('T3_init', 'T2_S4'),
    ('T3_init', 'T1_S6'), ('T3_init', 'T0_S8'), ('T3_init', 'accept_S10'),
    ('T3_S2', 'T3_S2'), ('T3_S2', 'T3_init'), ('T3_S2', 'T2_S4'),
    ('T3_S2', 'T1_S6'), ('T3_S2', 'T0_S8'), ('T2_S4', 'T2_S3'),
    ('T2_S4', 'T2_S4'), ('T2_S4', 'T1_S6'), ('T2_S4', 'T0_S8'),
    ('T2_S4', 'accept_S10'), ('T2_S3', 'T2_S3'), ('T2_S3', 'T2_S4'),
    ('T2_S3', 'T1_S6'), ('T2_S3', 'T0_S8'), ('T1_S6', 'T1_S5'),
    ('T1_S6', 'T1_S6'), ('T1_S6', 'T0_S8'), ('T1_S6', 'accept_S10'),
    ('T1_S5', 'T1_S5'), ('T1_S5', 'T1_S6'), ('T1_S5', 'T0_S8'),
    ('T0_S8', 'T0_S7'), ('T0_S8', 'T0_S8'), ('T0_S8', 'accept_S10'),
    ('T0_S7', 'T0_S7'), ('T0_S7', 'T0_S8'), ('accept_S10', 'accept_S9'),
    ('accept_S10', 'accept_S10'), ('accept_S9', 'accept_S9'),
    ('accept_S9', 'accept_S10')
]

# 构建图
graph = build_graph(edges)

# 查找所有路径
start_node = 'T3_init'
end_node = 'accept_S9'
all_paths = find_all_paths_iterative(graph, start_node, end_node)

print(f"All paths from {start_node} to {end_node}:")
for path in all_paths:
    print(path)

print(f"Total number of paths: {len(all_paths)}")

