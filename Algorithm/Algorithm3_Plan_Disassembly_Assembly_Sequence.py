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
    #3x2
    init_state_position = np.array([[-w / 2, -w], [-w / 2, 0], [-w / 2, w], [w / 2, -w], [w / 2, 0], [w / 2, w]])
    init_error_idx = np.array([1, 1, 0, 1, 1, 1])
    # #5x2
    # init_state_position = np.array([[-w, -w*2], [-w, -w], [-w, 0], [-w, w], [-w, w*2], [0, -w*2], [0, -w], [0, 0], [0, w], [0, w*2]])
    # init_error_idx = np.array([0, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    #3x3
    # init_state_position = np.array([[-w, -w], [-w, 0], [-w, w], [0, -w], [0, 0], [0, w], [w, -w], [w, 0], [w, w]])
    # init_error_idx = np.array([1, 1, 1, 1, 1, 1, 1, 0, 1])

    G = Quadrotor_graph(len(init_error_idx), init_state_position, init_error_idx, w)
    w = G.w

    # assuming that final state and minimum controllable assembly have been calculated
    #3x2
    final_state_position = np.array([[-w/2, -w], [-w/2, 0], [-w/2, w], [w/2, -w], [w/2, 0], [w/2, w]])
    final_error_idx = np.array([1, 0, 1, 1, 1, 1])
    # #5x2
    # final_state_position = np.array([[-w, -w*2], [-w, -w], [-w, 0], [-w, w], [-w, w*2], [0, -w*2], [0, -w], [0, 0], [0, w], [0, w*2]])
    # final_error_idx = np.array([1, 1, 0, 1, 1, 1, 1, 1, 1, 1])
    #3x3
    # final_state_position = np.array([[-w, -w], [-w, 0], [-w, w], [0, -w], [0, 0], [0, w], [w, -w], [w, 0], [w, w]])
    # final_error_idx = np.array([1, 1, 1, 1, 0, 1, 1, 1, 1])

    # init_error = np.array([0, 1, 1, 1, 1, 1])

    # minimum_assembly_idx = np.array([4, 5, 7, 8])
    minimum_assembly_idx = np.array([0, 1, 3, 4])

    print('initial cm value: %f' % CM.main(G.is_working, G.coord[:, 0], G.coord[:, 1]))
    # minn_controllability.calc_minn_ctrl_assmbly(G)

    temp_G = Quadrotor_graph(len(init_error_idx), init_state_position, init_error_idx, w)
    final_G = single_arrangement.reconfigurate_sequence(temp_G, final_state_position, final_error_idx, minimum_assembly_idx, 'dynamic')
    # final_G = regular_arrangement.reconfigurate_sequence(G,
    #                                                 final_state_position,
    #                                                 final_error_idx,
    #                                                 minimum_assembly_idx,
    #                                                 'dynamic')

