# import rospy
import yaml
from networkx.classes.digraph import DiGraph

# Import TS and action attributes from file
def import_ts_from_file(transition_system_textfile):
    try:
        with open(transition_system_textfile, 'r') as file:
            # Get dictionary from yaml text file
            transition_system = yaml.safe_load(file)
            # TODO: Add a proper parse and check (dimensions, attr,...)
            return transition_system
    except:
        # print(transition_system_textfile)
        # transition_system = yaml.safe_load(transition_system_textfile)
        raise ValueError("cannot load transition system from textfile")

def state_models_from_ts(TS_dict, initial_states_dict=None):
    state_models = []

    # If initial states are given as argument
    if initial_states_dict:
        # Check that dimensions are conform
        if (len(initial_states_dict.keys()) != len(TS_dict['state_dim'])):
            raise ValueError("initial states don't match TS state models: "+str(len(initial_states_dict.keys()))+" initial states and "+str(len(TS_dict['state_dim']))+" state models")

    # For every state model define in file, using state_dim to ensure order (dict are not ordered)
    for model_dim in TS_dict['state_dim']:
        state_model_dict = TS_dict['state_models'][model_dim]
        state_model = DiGraph(initial=set(), ts_state_format=[str(model_dim)])
        #------------------
        # Create all nodes
        #------------------
        for node in state_model_dict['nodes']:
            state_model.add_node(tuple([node]), label=set([str(node)]))
        # If no initial states in arguments, use initial state from TS dict
        if not initial_states_dict:
            state_model.graph['initial']=set([tuple([state_model_dict['initial']])])
        else:
            state_model.graph['initial']=set([tuple([initial_states_dict[model_dim]])])
        #----------------------------------
        # Connect previously created nodes
        #----------------------------------
        # Go through all nodes
        for node in state_model_dict['nodes']:
            # Go through all connected node
            for connected_node in state_model_dict['nodes'][node]['connected_to']:
                # Add edge between node and connected node
                # Get associated action from "connected_to" tag of state node
                act = TS_dict['state_models'][model_dim]["nodes"][node]['connected_to'][connected_node]
                # Use action to retrieve weight and guard from action dictionnary
                act_guard = TS_dict['actions'][act]['guard']
                act_weight = TS_dict['actions'][act]['weight']
                state_model.add_edge(tuple([node]), tuple([connected_node]), action = act, guard = act_guard, weight = act_weight)
        #-------------------------
        # Add state model to list
        #-------------------------
        state_models.append(state_model)

    return state_models

def state_models_from_ts_novel(TS_dict, Func_dict, initial_states_dict=None):
    state_models_in_total = []
    state_models = []

    # If initial states are given as argument
    if initial_states_dict:
        # Check that dimensions are conform
        if (len(initial_states_dict.keys()) != len(TS_dict['state_dim'])):
            raise ValueError("initial states don't match TS state models: "+str(len(initial_states_dict.keys()))+" initial states and "+str(len(TS_dict['state_dim']))+" state models")

    # For every state model define in file, using state_dim to ensure order (dict are not ordered)
    # for model_dim in TS_dict['state_dim']:
    #     state_model_dict = TS_dict['state_models'][model_dim]
    #     state_model = DiGraph(initial=set(), ts_state_format=[str(model_dim)])
    #     #------------------
    #     # Create all nodes
    #     #------------------
    #     for node in state_model_dict['nodes']: # 先录入所有的点
    #         state_model.add_node(tuple([node]), label=set([str(node)]))
    #     # If no initial states in arguments, use initial state from TS dict
    #     #----------------------------------
    #     # Connect previously created nodes
    #     #----------------------------------
    #     # Go through all nodes
    #     for node in state_model_dict['nodes']:
    #         # Go through all connected node
    #         for connected_node in state_model_dict['nodes'][node]['connected_to']:
    #             # Add edge between node and connected node
    #             # Get associated action from "connected_to" tag of state node
    #             act = TS_dict['state_models'][model_dim]["nodes"][node]['connected_to'][connected_node]
    #             # Use action to retrieve weight and guard from action dictionnary
    #             act_guard = TS_dict['actions'][act]['guard']
    #             act_weight = TS_dict['actions'][act]['weight']
    #             state_model.add_edge(tuple([node]), tuple([connected_node]), action = act, guard = act_guard, weight = act_weight)
        #-------------------------
        # Add state model to list
        #-------------------------
        # state_models.append(state_model)
    # print('=========print of func_dict=============')
    # print(Func_dict)
    # print('=============print over=================')
    for func_dim in Func_dict['state_dim']:
        for model_dim in TS_dict['state_dim']:
            state_model_dict = TS_dict['state_models'][model_dim]
            state_model = DiGraph(initial=set(), ts_state_format=[str(model_dim)])
            #------------------
            # Create all nodes
            #------------------
            for node in state_model_dict['nodes']: # 先录入所有的点
                state_model.add_node(tuple([node]), label=set([str(node)]))
            # If no initial states in arguments, use initial state from TS dict
            #----------------------------------
            # Connect previously created nodes
            #----------------------------------
            # Go through all nodes
            for node in state_model_dict['nodes']:
                # Go through all connected node
                for connected_node in state_model_dict['nodes'][node]['connected_to']:
                    # Add edge between node and connected node
                    # Get associated action from "connected_to" tag of state node
                    act = TS_dict['state_models'][model_dim]["nodes"][node]['connected_to'][connected_node]
                    # Use action to retrieve weight and guard from action dictionnary
                    act_guard = TS_dict['actions'][act]['guard']
                    act_weight = TS_dict['actions'][act]['weight']
                    state_model.add_edge(tuple([node]), tuple([connected_node]), action = act, guard = act_guard, weight = act_weight)


        func_model_dict = Func_dict['state_models'][func_dim]
        func_model = DiGraph(initial=set(), ts_state_format=[str(func_dim)])
        for node in func_model_dict['nodes']:
            func_model.add_node(tuple([node]), label=set([str(node)]))
        if not initial_states_dict:
            state_model.graph['initial']=set([tuple([func_model_dict['initial_position']])])
            state_models.append(state_model)
            # print(state_model.graph['initial'])
            func_model.graph['initial']=set([tuple([func_model_dict['initial_state']])])
        else:
            func_model.graph['initial']=set([tuple([initial_states_dict[func_dim]])])
        for node in func_model_dict['nodes']:
            for connected_node in func_model_dict['nodes'][node]['connected_to']:
                act = Func_dict['state_models'][func_dim]["nodes"][node]['connected_to'][connected_node]
                act_guard = Func_dict['actions'][func_dim][act]['guard']
                act_weight = Func_dict['actions'][func_dim][act]['weight']
                func_model.add_edge(tuple([node]), tuple([connected_node]), action = act, guard = act_guard, weight = act_weight)

        state_models.append(func_model)
        state_models_in_total.append(state_models)
        state_models = []
        func_model = []

    return state_models_in_total


def handle_ts_state_msg(ts_state_msg):
    # Extract TS state from request message
    # If only 1-dimensional state, state is directly a string
    #   in this case create tuple from the state from message array without the function tuple() 
    #   to avoid conversion from string to tuple of char
    if not (len(ts_state_msg.states) == len(ts_state_msg.state_dimension_names)):
        raise ValueError("Received TS states don't match TS state models: "+str(len(ts_state_msg.states))+" initial states and "+str(len(ts_state_msg.state_dimension_names))+" state models")
    elif len(ts_state_msg.states) > 1:
        ts_state = tuple(ts_state_msg.states)
        return ts_state
    elif len(ts_state_msg.states) == 1:
        ts_state = (ts_state_msg.states[0],)
        return ts_state
    else:
        raise ValueError("received empty TS state")

    #TODO Add check for message malformed (not corresponding fields)