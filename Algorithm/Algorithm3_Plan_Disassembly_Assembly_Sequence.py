import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import numpy as np
from quadrotor_graph import Quadrotor_graph
import single_arrangement
import regular_arrangement
import CM
w = single_arrangement.w


if __name__ == '__main__':
    # Define which case to run: '3x2' or '3x3'
    case = '3x2'  # Change this to '3x2' or '3x3' as needed

    if case == '3x2':
        # 3x2 configuration
        init_state_position = np.array([[-w / 2, -w], [-w / 2, 0], [-w / 2, w], [w / 2, -w], [w / 2, 0], [w / 2, w]])
        init_error_idx = np.array([1, 1, 0, 1, 1, 1])

        final_state_position = np.array([[-w / 2, -w], [-w / 2, 0], [-w / 2, w], [w / 2, -w], [w / 2, 0], [w / 2, w]])
        final_error_idx = np.array([1, 0, 1, 1, 1, 1])

    elif case == '3x3':
        # 3x3 configuration
        init_state_position = np.array([[-w, -w], [-w, 0], [-w, w], [0, -w], [0, 0], [0, w], 
                                        [w, -w], [w, 0], [w, w]])
        init_error_idx = np.array([1, 1, 1, 1, 1, 1, 1, 0, 1])

        final_state_position = np.array([[-w, -w], [-w, 0], [-w, w], [0, -w], [0, 0], [0, w], 
                                        [w, -w], [w, 0], [w, w]])
        final_error_idx = np.array([1, 1, 1, 1, 0, 1, 1, 1, 1])

    # Initialize the graph based on the chosen configuration
    G = Quadrotor_graph(len(init_error_idx), init_state_position, init_error_idx, w)
    w = G.w

    # init_error = np.array([0, 1, 1, 1, 1, 1])
    # minimum_assembly_idx = np.array([4, 5, 7, 8])
    minimum_assembly_idx = np.array([0, 1, 3, 4])

    print('initial cm value: %f' % CM.main(G.is_working, G.coord[:, 0], G.coord[:, 1]))
    # minn_controllability.calc_minn_ctrl_assmbly(G)
    temp_G = Quadrotor_graph(len(init_error_idx), init_state_position, init_error_idx, w)
    # full disassembly
    final_G = single_arrangement.reconfigurate_sequence(temp_G, final_state_position, final_error_idx, minimum_assembly_idx, 'dynamic')
    # partial disassembly
    final_G = regular_arrangement.reconfigurate_sequence(G,
                                                    final_state_position,
                                                    final_error_idx,
                                                    minimum_assembly_idx,
                                                    'dynamic')