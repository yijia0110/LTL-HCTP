# -*- coding: utf-8 -*-
# import rospy
from inspect import stack
from os import path
import sys
from tkinter import N

from networkx import jaccard_coefficient
from ltl_automaton_planner.ltl_tools.ltl2ba import run_ltl2ba
from ltl_automaton_planner.ltl_tools.promela import parse as parse_ltl, find_states, find_symbols
from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard
from itertools import product as cartesian_product

from networkx.classes.digraph import DiGraph
import time

import networkx as nx
from collections import defaultdict
###########################################################
# Construct a buchi automation given the LTL specification
# all the graph search happens here
# Input is the LTL formula
###########################################################

def buchi_from_ltl(formula,Type):
    promela_string = run_ltl2ba(formula)
    symbols = find_symbols(formula)
    edges, accepts, inits = parse_ltl(promela_string)
    
    accepting_words = 0.0
    start_time = time.time()
    print("=========新加的计算量==========")
    edge = [tuple(edge) for edge in edges.keys()]
    paths = list()
    for init_temp in inits:
        for accept_temp in accepts:
            graph = build_graph2(edge)
            all_paths, accepting_words = find_all_paths_iterative(graph, init_temp, accept_temp)

            paths.append(all_paths)
    #         print("==========从{}到{}的路径共有{}条，accepting_words{}个，详细信息为==========".
    #               format(init_temp, accept_temp, len(all_paths), accepting_words))
    last_time = time.time()
    print("=========新加计算时间为{}==========".format(last_time - start_time))

    (states, initials, accepts) = find_states(edges)
    buchi = DiGraph(type=Type, initial=initials, accept=accepts, symbols=symbols)
    for state in states:
        buchi.add_node(state)
    for (ef,et) in edges.keys():
        guard_formula = edges[(ef,et)]
        guard_expr = parse_guard(guard_formula)
        buchi.add_edge(ef, et, guard=guard_expr, guard_formula=guard_formula)

    print('LTL Planner: full Buchi constructed has %d states, %s edges and %s accepting words.' 
          %(len(buchi.nodes()), len(buchi.edges()), int(accepting_words)))
    return buchi

def mission_to_buchi(hard_spec, soft_spec): # 先调用的是这个，然后转到DuoBA_from_ltls
    if (hard_spec and not soft_spec):
        buchi = buchi_from_ltl(hard_spec,'hard_buchi')
    elif (soft_spec and not hard_spec):
        buchi = buchi_from_ltl(soft_spec,'soft_buchi')
    elif (hard_spec and soft_spec):
        buchi = DuoBA_from_ltls(hard_spec, soft_spec)

    # edge = list(set([tuple(edge) for edge in buchi.edges().keys()]))
    # accepts = [edge[1] for edge in buchi.edges().keys() if "accept" in edge[1]]
    # inits = [edge[0] for edge in buchi.edges().keys() if "init" in edge[0]]

    # paths = list()
    # # accepting_words = 0.0
    # for init_temp in inits:
    #     for accept_temp in accepts:
    #         graph = build_graph2(edge)
    #         all_paths, accepting_words = find_all_paths_iterative(graph, init_temp, accept_temp)

    #         paths.append(all_paths)
            # print("==========从{}到{}的路径共有{}条，accepting_words{}个，详细信息为==========".
            #       format(init_temp, accept_temp, len(all_paths), accepting_words))

    # print('LTL Planner: full Buchi constructed has %d states and %s edges' %(len(buchi.nodes()), len(buchi.edges())))
    return buchi

def DuoBA_from_ltls(hard_spec, soft_spec): # 同时提供了hard和soft时，采用的LTL2BA程序
    hard_buchi = buchi_from_ltl(hard_spec, 'hard_buchi') # ltl2buchi for hard specs
    soft_buchi = buchi_from_ltl(soft_spec, 'soft_buchi') # ltl2buchi for soft specs
    hard_symbols = hard_buchi.graph['symbols'] # 应该是将所有点都进行了提取
    soft_symbols = soft_buchi.graph['symbols']
    symbols = set(hard_symbols).union(set(soft_symbols)) # soft和hard进行组合
    DuoBA = DiGraph(type='safe_buchi', hard=hard_buchi, soft=soft_buchi, symols=symbols) # 初次赋值
    initial = set()
    accept = set()
    for (h_node, s_node, l) in cartesian_product(hard_buchi.nodes(), soft_buchi.nodes(), [1, 2]): # 在这里完成了节点的标记，1代表initial(soft)和accept(hard)，2没有显示，但是应该与1是补集
        DuoNode = (h_node, s_node, l)
        DuoBA.add_node(DuoNode,hard=h_node, soft=s_node, level=l)# 这里的赋值规则绝对不可能赋值两次，肯定是只赋值一次的那种类型，前面是数据，后面是赋值方式
        if (h_node in hard_buchi.graph['initial'] and 
            s_node in soft_buchi.graph['initial'] and l == 1):
            initial.add(DuoNode)
        if (h_node in hard_buchi.graph['accept'] and l == 1):
            accept.add(DuoNode)
    DuoBA.graph['accept'] = accept # 再次赋值
    DuoBA.graph['initial'] = initial
    for f_duonode in DuoBA.nodes():
        for t_duonode in DuoBA.nodes():
            f_h_node, f_s_node, f_level = check_duo_attr(DuoBA, f_duonode)
            t_h_node, t_s_node, t_level = check_duo_attr(DuoBA, t_duonode)
            if (t_h_node not in DuoBA.graph['hard'].neighbors(f_h_node) or 
                t_s_node not in DuoBA.graph['soft'].neighbors(f_s_node)): # 如果二者不相邻，则直接跳过
                continue
                # relaxed because no common input alphabets are enabled
            hardguard = DuoBA.graph['hard'].edges[f_h_node,t_h_node]['guard'] # 如果相邻，创建hard guard连接
            softguard = DuoBA.graph['soft'].edges[f_s_node,t_s_node]['guard'] # 如果不相邻，创建soft guard连接
            if ((f_h_node not in DuoBA.graph['hard'].graph['accept'] and 
                f_level == 1 and t_level == 1) or 
                (f_h_node in DuoBA.graph['hard'].graph['accept'] and 
                f_level == 1 and t_level == 2) or 
                (f_s_node not in DuoBA.graph['soft'].graph['accept'] and  
                f_level == 2 and t_level == 2) or 
                (f_s_node in DuoBA.graph['soft'].graph['accept'] and 
                f_level == 2 and t_level == 1)): # 这里仅有2者异号的时候，才能为accept
                DuoBA.add_edge(f_duonode, t_duonode, hardguard=hardguard, softguard=softguard)
    return DuoBA

def check_duo_attr(DuoBA, node):
    return DuoBA.nodes[node]['hard'], DuoBA.nodes[node]['soft'], DuoBA.nodes[node]['level']

def check_label_for_buchi_edge(buchi, label, f_buchi_node, t_buchi_node):
    buchi_type = buchi.graph['type']
    if buchi_type == 'hard_buchi':
        truth = buchi.edges[f_buchi_node, t_buchi_node]['guard'].check(label)
        dist = 0
    if buchi_type == 'soft_buchi':
        truth = True
        dist = buchi.edges[f_buchi_node, t_buchi_node]['guard'].distance(label)
    if buchi_type == 'safe_buchi':
        truth = buchi.edges[f_buchi_node, t_buchi_node]['hardguard'].check(label)
        if truth:
            dist = buchi.edges[f_buchi_node, t_buchi_node]['softguard'].distance(label)
        else:
            dist = 1000
    return truth, dist

def iter_find(edges, init_temp, accept_temp, counter):
    stack = [(init_temp, [init_temp])]  # (当前节点, 路径)
    all_paths = []  # 用于存储所有路径

    while stack:
        current, path = stack.pop() # 弹出栈顶元素
        if current == accept_temp:  # 如果当前节点是终点，将路径添加到结果中
            counter = counter + 1
            all_paths.append(path)
        # for edge in edges:
            if edges[0][0] == current:
                next_node = edges[0][1]
                if next_node not in path:  # 避免回路
                    stack.append((next_node, path + [next_node]))  # 将下一个节点和路径入栈
                    counter = counter + 1
    return all_paths, counter

def iter_find2(edges, init_temp, accept_temp, counter):
    stack = list()
    stack_temp = [edge for edge in edges if edge[0] == init_temp]
    counter += len(stack_temp)
    stack.append(stack_temp)

    all_paths = []
    # all_paths.append(init_temp)
    while stack:
        print("上一次已结束")
        input()
        print(len(stack))
        stack_temp = stack.pop()
        all_paths.append(stack_temp[-1][0])
        # all_paths.append(init_temp)
        print(all_paths)
        # last_stack = []
        for stacktt in stack_temp:
            print(stacktt)
            if accept_temp in stacktt[1] or stacktt[0] == stacktt[1]:
                # print("已经找到了accept或自循环")
                continue
            # else:
            #     print(stacktt)
            stack_temp2 = [edge for edge in edges if edge[0] == stacktt[1]]
            counter += len(stack_temp2)

            stack.append(stack_temp2)
        # all_paths.append(stacktt[1])
            # if 
            #     last_stack = stacktt[0]
        print(counter)
    return counter

def iter_find3(edge_dict, init_temp, accept_temp, counter):
    init_hi = init_temp
    # dict_temp = edge_dict[init_hi]

    paths = dict() # 用来存储所有路径
    path_temp = list()
    path_temp.append(init_hi) # 路径的值
    path_index = 1 # 路径的键
    # index_temp = list()

    stack = dict()
    stack2 = dict()
    stack_temp = list()

    name_indexes = list()

    stack_temp.append(edge_dict[init_hi])
    print("==函数调用flag==")
    # while stack:
    while stack_temp:
        print("=====请求输入=====")
        input()
        stack_tt = stack_temp.pop() # 这里还是得用pop，要不这个引用会有问题
        pindex_temp = 0 # 用来搜索当前路径中path_temp中不存在的路径
        while pindex_temp < len(stack_tt) and stack_tt[pindex_temp] in path_temp:
            # 这里需要两个满足2个条件,1是索引要在范围内,2是已经在路径中了,如果不满足,那就没必要搜索了
            pindex_temp += 1
        
        pindex_temp = pindex_temp - 1

        if pindex_temp == len(stack_tt) - 1: # 对搜索出的索引进行判断,如果已经搜到了末尾
            init_hi = stack_tt[pindex_temp]
            if init_hi not in path_temp: # 在末尾,但是没有在路径中,可以加入路径
                path_temp.append(stack_tt[init_hi])

                name_indexes.append(init_hi)

                stack[init_hi] = pindex_temp # 更新edge字典的搜索索引
                stack2[init_hi] = edge_dict[init_hi] # 更新edge字典的值,以配合上述的索引
            else:
                continue
        else:
            init_hi = stack_tt[pindex_temp] # 如果没有搜索到末尾,那就直接进行赋值
            path_temp.append(init_hi)
            name_indexes.append(init_hi)

            stack[init_hi] = pindex_temp # edge字典的搜索索引
            stack2[init_hi] = edge_dict[init_hi]

        if (pindex_temp != len(stack_tt) - 1 or stack_tt[pindex_temp] not in path_temp) and accept_temp in stack_tt[pindex_temp]:
            # 如果已经搜索到的新的路径点,且accept能在路径中搜索到,那就更新上这条路径,并继续搜索别的
            paths[path_index] = path_temp
            print(paths)
            path_index += 1

            path_temp.pop()
            name_indexes.pop()

        stack_tt2 = edge_dict[stack_tt[pindex_temp]]
        stack_temp.append(stack_tt2)

    return counter, paths

def edge2dict(edge):
    edge_dict = dict()
    for edge_temp in edge:
        # print(edge_temp)
        if edge_temp[0] != edge_temp[1]:
            if edge_temp[0] in edge_dict:
                edge_dict[edge_temp[0]].append(edge_temp[1])
            else:
                edge_dict[edge_temp[0]] = []
                edge_dict[edge_temp[0]].append(edge_temp[1])
    return edge_dict

# 构建邻接表
def build_graph2(edges):
    graph = defaultdict(list)
    for start, end in edges:
        graph[start].append(end)
    return graph

# 使用迭代方法查找所有路径
def find_all_paths_iterative(graph, start, end):
    stack = [(start, [start])]  # 栈中保存当前节点和路径
    paths = []
    accepting_words = 0.0
    while stack:
        vertex, path = stack.pop()
        
        for next_node in graph[vertex]:
            if next_node not in path:  # 防止环
                new_path = path + [next_node]
                # if end not in next_node:
                accepting_words += 1
                if next_node == end:
                    paths.append(new_path)
                else:
                    stack.append((next_node, new_path))
                    
    return paths, accepting_words