import sys
sys.path.append("/home/zyj/ltl_codedown/ltl_automaton_planner/ltl_automaton_planner")
sys.path.append("/home/zyj/ltl_codedown/ltl_automaton_planner")
# sys.path.append("/home/zyj/ltl_codedown/ltl_automaton_planner/ltl_automaton_msgs")
import sys
import importlib
import yaml
import time
import os

from copy import deepcopy

import std_msgs

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.ltl_planner_multi_robot_exp import LTLPlanner_MultiRobot_Exp

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg, state_models_from_ts_novel

# Import LTL automaton message definitions
# from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
# from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse
from networkx.drawing.nx_agraph import to_agraph

from collections import defaultdict

#输入为Digraph
# 正式程序部分
def show_automaton(automaton_graph): # 原版程序可视化部分
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    plt.show()

    return

def show_automaton_novel(seqs, automaton_graph): # 自己写的可视化部分
    actions = seqs.action_sequence.items()
    colored = seqs.ts_state_sequence.items()
    fun_dicts = []
    script_path = os.path.dirname(__file__)

    func_reader = os.path.join(script_path, 'function_file3.txt')
    with open(func_reader, 'r') as file:
        real_counter = 0
        lines = file.readlines()
        for i in range(len(lines)):
            line = lines[i].strip()
            parts = line.split(',')
            fun_dict = {}
            counter = 1
            for part in parts:
                if counter <= 3 or (counter == len(parts) and counter <= 5):
                    print(part)
                    key, value = part.split(':')
                    key = key.strip()
                    value = value.strip()

                    fun_dict[key] = (value)
                
                else:
                    key, value1, value2 = part.split(':')
                    key = key.strip()
                    value1 = value1.strip()
                    value2 = value2.strip()

                    fun_dict[key] = (value1, value2)
                counter = counter + 1
            fun_dicts.append(fun_dict)

    pos=nx.circular_layout(automaton_graph)

    dict_color = dict(colored)
    dict_action = dict(actions)
    fig_list = []
    for i in range(len(colored)):
        dict_temp = dict_color[i]
        act_temp = dict_action[i]

        if len(act_temp) == 0:
            continue

        fun_temp = fun_dicts[i]
        keys_temp = list(fun_temp.keys())

        fig = plt.figure()
        plt.title("Planned task trajectory of UAV"+str(i))
        nx.draw(automaton_graph, pos)

        keys = []
        nodes = []
        G = nx.DiGraph()
        G_start = nx.DiGraph()
        G_close = nx.DiGraph()
        for j in range(len(dict_temp)):
            keys = keys + [(dict_temp[j][0], )]
            if j == len(dict_temp)-1:
                G_close.add_node(dict_temp[j][0])
            if j > 0:
                if dict_temp[j][0] != dict_temp[j-1][0]:
                    nodes.append(dict_temp[j][0])
                    G.add_edge(dict_temp[j][0], dict_temp[j-1][0], action=act_temp[j-1], guard=1,weight=10)
                else:
                    if act_temp[j-1] == 'standby':
                        G.add_edge(dict_temp[j][0], dict_temp[j-1][0], action=act_temp[j-1], guard=1,weight=0.01)
                    else:
                        G.add_edge(dict_temp[j][0], dict_temp[j-1][0], action=act_temp[j-1], guard=1,weight=10)
            else:
                G_start.add_node(dict_temp[j][0])
                nodes.append(dict_temp[j][0])
                if act_temp[j-1] == 'standby':
                    G.add_edge(dict_temp[j][0], dict_temp[j-1][0], action=act_temp[j-1], guard=1,weight=0.01)
                else:
                    G.add_edge(dict_temp[j][0], dict_temp[j-1][0], action=act_temp[j-1], guard=1,weight=10)

        G.add_nodes_from(nodes)

        selected_keys = keys
        selected_pos = {key: pos[key] for key in selected_keys}
        selected_pos2 = {key[0]: value for key,value in selected_pos.items()}
        selected_pos_start = {}
        selected_pos_start[selected_keys[0][0]] = selected_pos2[selected_keys[0][0]]

        selected_pos_close = {}
        selected_pos_close[selected_keys[0][0]] = selected_pos2[selected_keys[-1][0]]

        nx.draw(G, selected_pos2,node_color=(255/255,80/255,80/255), edge_color='red')
        nx.draw(G_start, selected_pos2,node_color=(102/255,255/255,204/255), edge_color='red')
        nx.draw(G_close, selected_pos2,node_color=(255/255,153/255,0/255), edge_color='red')

        nx.draw_networkx_labels(automaton_graph, pos, font_color='black', font_family='cambria math')

        edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
        edge_labels_new = nx.get_edge_attributes(G, 'action')

        nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
        nx.draw_networkx_edge_labels(G, selected_pos2, edge_labels = edge_labels_new, font_color='red')


    # title = "Planned task trajectory results"
    # plt.title(title)
    # 思路记录：直接打破原有的格局，将一个图automaton_graph拆分成染色和未染色两个部分，二者的点和连边均不完全相同，如果后续有显示错误的问题，可以再行修改。
    # 其中黑色部分可以直接将所有点都包含进去，但是连边不行，将红色部分控制的连边进行删除。
    # 或者可以这样，在进行绘制时，与stay相关的边可以删除一些，先用红色打底，然后黑色部分就不画了，这样突出一个红色的stay（上次出问题的也是这个）
        fig_list.append(fig)
    plt.show()
    return

class MultiRobot_Planner_Exp(object):
    def __init__(self):
        self.init_params() # 当前文件的内部函数
        # self.setup_pub_sub() # 当前文件的内部函数

    def run(self):
        self.build_automaton() # 构建自动机

    def init_params(self):
        self.agent_name_mobile_1 = "uav_0"
        self.initial_beta = 1000 #考虑软任务的重要性 越大 软任务就越重要
        self.gamma = 10 #前缀和后缀之间的成本比率 越大 后缀cost越小

        self.flag = 0 #标记发送过结果了


        # LTL task
        #----------
        # Get LTL hard task and raise error if don't exist
        # self.hard_task = "<>((p3 && repair) && ! (p3 && scan) && <> (p3 && scan)) && <> ((p21 && wash) && <> (mow && p21) && <> (scan && p21)) && <> ((p21 && sweep) && ! (p21 && wash) && <> (mow && p21)) && <> (fix && t5 && ! p18) && ! p24 U (p27 && sweep) && <> ((wash && p34) && [](p34 && scan))"
        # self.hard_task = "<>((c1 && repair) && !(c1 && scan) && <> (c1 && scan)) \
        # && <> ((c3 && wash) && <> (c3 && mow) && <> (c3 && scan)) && <> ((c3 && sweep) \
        # && ! (c3 && wash) && <> (p3 && mow)) && <> ((p5 && fix) && ! (c2 && fix)) \
        # && X (!(c4 && sweep) U (c5 && sweep)) && <> ((c6 && wash) && [](c6 && scan))"

        self.hard_task = "<>((c1 && repair) && !(c1 && scan) && <> (c1 && scan)) && <> ((c3 && wash) && <> (c3 && mow) && <> (c3 && scan)) && <> ((c3 && sweep) && ! (c3 && wash) && <> (p3 && mow)) && <> ((p5 && fix) && (!c2)) && <> ((!c4) U (c5 && sweep)) && <> ((c6 && wash) && [](c6 && scan))"

        # self.hard_task = "<>(desk && default && X((carrybin U dispose) && <>default))"
        # self.hard_task = "<>(c7 && training && <>(c6 && training && <> (c7 && training && <>(p4 && standby)))) && []((!p4 && X p4) -> training)"
        # self.hard_task = "<>(c7 && standby) && <>(p3 && scan)"
        self.soft_task = ""

        transition_system_textfiles = "config/ts_total.yaml"
        func_textfiles = "config/func_total2.yaml"

        self.transition_system_textfiles = import_ts_from_file(transition_system_textfiles)
        self.func_textfiles = import_ts_from_file(func_textfiles)
        # TS初值由反馈获取
        self.initial_ts_state_from_agent = False
        self.initial_state_ts_dict = None

        self.replan_on_unplanned_move = True
        self.check_timestamp = True

    def build_automaton(self):
        # 规划集中，执行分布（两层规划器，通信和更新的问题，就像之前看到的某篇）
        # Import state models from TS 
        # 是list,把yaml文件中所有state_dim分别建立为一个state_model

        ts_lists = state_models_from_ts_novel(self.transition_system_textfiles, self.func_textfiles, self.initial_state_ts_dict) # 初始化
        self.ts_list = [TSModel(ts) for ts in ts_lists] # 首次调用tsmodel，这里调用的时候，直接到底才切换是对的

        self.ltl_planner_multi_robot = LTLPlanner_MultiRobot_Exp(self.ts_list, self.hard_task, self.soft_task, self.initial_beta, self.gamma) # 进行初始化
        self.ltl_planner_multi_robot.task_allocate() # 正式进行任务分配
        self.robot_model_mobile_1 = self.ts_list[0]

#==============================
#             Main
#==============================
if __name__ == '__main__':
    multi_robot_ltl_planner_node = MultiRobot_Planner_Exp()
    multi_robot_ltl_planner_node.run()
