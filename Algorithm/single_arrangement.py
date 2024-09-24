import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import numpy as np
import CM
import minn_controllability
from quadrotor_graph import Quadrotor_graph
from quadrotor_graph import w
import imageio

# N = 9
# w = 0.53
ax_lim = 4
plt_time = 0.25
dir_expression = np.array([[-1, 0],[0, 1],[1, 0],[0, -1]])
k=500
# transform_step = []
images = []

visualization_type = {
    'none': 0,
    'static': 1,
    'dynamic': 2
}


def G_viz_static(G, cm_value, state, target_idx=-1):
    plt.xlim((-ax_lim, ax_lim))
    plt.ylim((-ax_lim, ax_lim))

    if state == 'disassemble':
        plt.title('disassembly: with target ' + str(target_idx) + ', cm = ' + str(cm_value))
    elif state == 'assemble':
        plt.title('assembly: move to ' + str(target_idx) + ', cm = ' + str(cm_value))
    else:
        plt.title('initial state, cm = ' + str(cm_value))

    for i in range(G.n):
        if G.is_working[i] == 0:
            plt.scatter(G.coord[i, 1], -G.coord[i, 0], s=w*k, c='r', marker='s')
        else:
            plt.scatter(G.coord[i, 1], -G.coord[i, 0], s=w*k, c='g', marker='s')

        for dir in range(4):
            if G.edge[i, dir] != -1:
                j = int(G.edge[i, dir])
                xpoints = [G.coord[i][1], G.coord[j][1]]
                ypoints = [-G.coord[i][0], -G.coord[j][0]]
                plt.plot(xpoints, ypoints)

        plt.annotate(str(i), xy=(G.coord[i, 1], -G.coord[i, 0]), xytext=(G.coord[i, 1], -G.coord[i, 0]+0.1))
    plt.show()


def G_viz_dynamic(err, transform_step, transform_idx, graph_step):
    plt.ion()
    step = (len(graph_step) - 2)*2
    frequency = 3
    for step_idx in range(step):
        if step_idx%2 == 0:     # for disassembly stage
            for num in range(frequency):
                plt.cla()
                cnt = step_idx
                sector = num + 1
                edge = graph_step[int(cnt / 2) + 1]

                for i in range(len(graph_step[0])):
                    for dir in range(4):
                        if edge[i, dir] > -1:
                            j = int(edge[i, dir])
                            xpoints = [transform_step[cnt, i, 1], transform_step[cnt, j, 1]]
                            ypoints = [-transform_step[cnt, i, 0], -transform_step[cnt, j, 0]]
                            plt.plot(xpoints, ypoints)


                for i in range(len(graph_step[0])):
                    if i == transform_idx[cnt]:
                        continue
                    if i == err:
                        plt.scatter(transform_step[cnt, i, 1], -transform_step[cnt, i, 0], s=w * k, c='r', marker='s')
                    else:
                        plt.scatter(transform_step[cnt, i, 1], -transform_step[cnt, i, 0], s=w * k, c='g', marker='s')
                    plt.annotate(str(i+1), xy=(transform_step[cnt, i, 1], -transform_step[cnt, i, 0]),
                                 xytext=(transform_step[cnt, i, 1], -transform_step[cnt, i, 0] + 0.1))

                moving = int(transform_idx[cnt])
                posi = np.array(transform_step[cnt, moving, :])
                delta = (transform_step[cnt + 1, moving, :] - transform_step[cnt, moving, :])
                posi += delta / frequency * sector

                plt.scatter(posi[1], -posi[0], s=w * k, marker='s', c='g')
                plt.annotate(str(moving+1), xy=(posi[1], -posi[0]), xytext=(posi[1], -posi[0] + 0.1))
                plt.xlim((-ax_lim, ax_lim))
                plt.ylim((-ax_lim, ax_lim))
                
                #Gif
                image_filename = f'disassembly_frame_{step_idx}_{num}.png'
                plt.savefig(image_filename)
                images.append(image_filename)
                plt.pause(plt_time)

        else:                       # for assembly stage
            # assembly trajectory is divided into 3 parts.
            moving = int(transform_idx[step_idx])
            final_pos = np.array([transform_step[step_idx+1, moving, 0], transform_step[step_idx+1, moving, 1]])

            # decide trajectory sequence
            chk_pos = np.array(transform_step[step_idx, :, :])
            occupied_up = np.where((chk_pos == (final_pos[0] + dir_expression[0, 0]*w,
                                                final_pos[1] + dir_expression[0, 1]*w)).all(axis=1))
            occupied_right = np.where((chk_pos == (final_pos[0] + dir_expression[1, 0]*w,
                                                   final_pos[1] + dir_expression[1, 1]*w)).all(axis=1))
            occupied_down = np.where((chk_pos == (final_pos[0] + dir_expression[2, 0]*w,
                                                  final_pos[1] + dir_expression[2, 1]*w)).all(axis=1))
            occupied_left = np.where((chk_pos == (final_pos[0] + dir_expression[3, 0]*w,
                                                  final_pos[1] + dir_expression[3, 1]*w)).all(axis=1))

            target_pos = [np.array([transform_step[step_idx, moving, 0], transform_step[step_idx, moving, 1]])]
            temp = transform_step[step_idx, moving, :]

            center = np.array([np.sum(transform_step[step_idx, :, 0]), np.sum(transform_step[step_idx, :, 1])])
            center = center - transform_step[step_idx, moving, :]
            center = center/(len(graph_step[0])-1)

            chk_pos[moving, :] = center

            if not np.size(occupied_down):
                boundary = np.max(chk_pos[:, 0]) + 2 * w
                if temp[0] < boundary:
                    temp[0] = boundary

            if not np.size(occupied_up):
                boundary = np.min(chk_pos[:, 0]) - 2 * w
                if temp[0] > boundary:
                    temp[0] = boundary
            target_pos.append(np.array(temp))

            if not np.size(occupied_left):
                boundary = np.min(chk_pos[:, 1]) + 2 * w
                if temp[1] > boundary:
                    temp[1] = boundary

            if not np.size(occupied_right):
                boundary = np.max(chk_pos[:, 1]) - 2 * w
                if temp[1] < boundary:
                    temp[1] = boundary
            target_pos.append(np.array(temp))


            # for now the moving unit is on the same side as target position
            if not np.size(occupied_up) or not np.size(occupied_down):
                target_pos.append(np.array([temp[0], final_pos[1]]))
                target_pos.append(np.array([final_pos[0], final_pos[1]]))
            else:
                target_pos.append(np.array([final_pos[0], temp[1]]))
                target_pos.append(np.array([final_pos[0], final_pos[1]]))

            for list_process in range(len(target_pos)-1):
                for num in range(frequency):
                    plt.cla()
                    cnt = list_process
                    sector = num + 1
                    edge = graph_step[int(step_idx / 2) + 1]

                    for i in range(len(graph_step[0])):
                        for dir in range(4):
                            if edge[i, dir] > -1:
                                j = int(edge[i, dir])
                                xpoints = [transform_step[step_idx, i, 1], transform_step[step_idx, j, 1]]
                                ypoints = [-transform_step[step_idx, i, 0], -transform_step[step_idx, j, 0]]
                                plt.plot(xpoints, ypoints)

                    for i in range(len(graph_step[0])):
                        if i == transform_idx[step_idx]:
                            continue
                        if i == err:
                            plt.scatter(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0], s=w * k, c='r',
                                        marker='s')
                        else:
                            plt.scatter(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0], s=w * k, c='g',
                                        marker='s')
                        plt.annotate(str(i+1), xy=(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0]),
                                     xytext=(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0] + 0.1))
                        

                    posi = np.array(target_pos[cnt])
                    delta = (target_pos[cnt+1] - target_pos[cnt])
                    posi += delta / frequency * sector

                    plt.scatter(posi[1], -posi[0], s=w * k, marker='s', c='g')
                    plt.annotate(str(moving+1), xy=(posi[1], -posi[0]), xytext=(posi[1], -posi[0] + 0.1))
                    
                    plt.xlim((-ax_lim, ax_lim))
                    plt.ylim((-ax_lim, ax_lim))

                    #Gif
                    image_filename = f'assembly_frame_{step_idx}_{list_process}_{num}.png'
                    plt.savefig(image_filename)
                    images.append(image_filename)
                    plt.pause(plt_time)

    plt.cla()
    edge = graph_step[int(step / 2) + 1]
    for i in range(len(graph_step[0])):
        if i == err:
            plt.scatter(transform_step[step, i, 1], -transform_step[step, i, 0], s=w * k, c='r', marker='s')
        else:
            plt.scatter(transform_step[step, i, 1], -transform_step[step, i, 0], s=w * k, c='g', marker='s')
        plt.annotate(str(i+1), xy=(transform_step[step, i, 1], -transform_step[step, i, 0]),
                     xytext=(transform_step[step, i, 1], -transform_step[step, i, 0] + 0.1))
        for dir in range(4):
            j = int(edge[i, dir])
            if j > 0:
                xpoints = [transform_step[step, i, 1], transform_step[step, j, 1]]
                ypoints = [-transform_step[step, i, 0], -transform_step[step, j, 0]]
                plt.plot(xpoints, ypoints)
    plt.xlim((-ax_lim, ax_lim))
    plt.ylim((-ax_lim, ax_lim))
    image_filename = f'final_frame.png'
    plt.savefig(image_filename)
    images.append(image_filename)
    plt.pause(plt_time*10)

    #Gif
    gif_name='full_self_reconfiguration.gif'
    # Generate GIF
    with imageio.get_writer(gif_name, mode='I', duration=0.5) as writer:
        for filename in images:
            image = imageio.imread(filename)
            writer.append_data(image)
    print(f"GIF saved as {gif_name}")

def reconfigurate_sequence(init_G, final_pos, final_err_pos, minn_assembly_idx, is_visualization='static'):
    # coordinate normalization
    """
    :param init_G: graph of initial assembly
    :param final_pos: graph of final assembly
    :param final_err_pos: current state of corresponding position
    :param minn_assembly_idx: the index of units of the minimum controllable assembly
    :return: result of the final assembly

    IMPORTANT: no meaning of the idx of final_pos and final_err_pos
    """
    if is_visualization == 'static':
        G_viz_static(init_G, CM.main(init_G.is_working, init_G.coord[:, 0], init_G.coord[:, 1]), 'initial')
    transform_step = np.zeros([100, len(final_err_pos), 2])
    transform_step[0, :, :] = np.array(init_G.coord)
    graph_step = []
    transform_idx = np.zeros(100)

    graph_step.append(np.array(init_G.edge))
    '''
    initialization: 
        1. match coordinate of initial and final state
        2. set graph for final state
    '''
    num = init_G.n
    final_G = Quadrotor_graph(num, final_pos, final_err_pos, w)
    init_failed_idx = np.where(init_G.is_working == 0)
    init_failed_idx = int(init_failed_idx[0])
    final_failed_idx = np.where(final_err_pos == 0)
    final_failed_idx = int(final_failed_idx[0])

    final_pos = final_pos - (final_pos[final_failed_idx] - init_G.coord[init_failed_idx])

    final_available_pos_list = list()
    detaching_list = list()
    for i in range(num):
        final_available_pos_list.append(i)
        detaching_list.append(i)

    isin_minn_assembly = np.zeros(num)
    for i in range(len(minn_assembly_idx)):
        isin_minn_assembly[minn_assembly_idx[i]] = 1
    final_available_pos_list.remove(final_failed_idx)

    '''step 1. traverse the combination of units to calculate available list of final state'''
    # assumption: in final assembly minn controllable assembly can still be found.
    # find positions in final state that haven't matched a quadrotor in init_G
    traverse_list = Queue()
    traverse_list.put(init_failed_idx)
    traverse_list.put(final_failed_idx)
    is_traverse = np.zeros(num)

    curr_assembled_pos = list()
    curr_final_error_idx = list()
    curr_assembled_pos.append(final_pos[final_failed_idx])
    curr_final_error_idx.append(final_err_pos[final_failed_idx])

    while not traverse_list.empty():
        curr_idx = traverse_list.get()
        detaching_list.remove(curr_idx)
        final_idx = traverse_list.get()
        is_traverse[curr_idx] = 1
        for direction in range(4):
            to_idx = int(init_G.edge[curr_idx, direction])
            if to_idx != -1 and is_traverse[to_idx] == 0:
                if final_G.edge[final_idx, direction] == -1:
                    # print('cannot match units in final assembly')
                    continue

                # to_idx in minn assembly
                final_to_idx = int(final_G.edge[final_idx, direction])
                is_traverse[to_idx] = 1
                traverse_list.put(to_idx)
                traverse_list.put(final_to_idx)

                # set corresponding position in final state unavailable
                final_available_pos_list.remove(final_to_idx)

                # calculate current assembled position list
                curr_assembled_pos.append(final_pos[final_to_idx])
                curr_final_error_idx.append(final_err_pos[final_to_idx])
    remaining_num = len(detaching_list)
    '''step 2. calculate dis/assembling sequence'''
    for i in range(remaining_num):
        '''disassembly process'''
        # for disassembly, select index j s.t. the cm value of remaining units is maximized
        disassemble_idx = -1
        max_cm = 0
        for j in range(len(detaching_list)):
            curr_idx = detaching_list[j]

            if np.sum(init_G.edge[curr_idx, :] >= 0) >= 3:   # check with edges
                continue

            init_pos_list = list(init_G.coord)
            init_err_list = list(init_G.is_working)

            init_pos_list.pop(curr_idx)
            init_err_list.pop(curr_idx)
            init_pos_list = np.array(init_pos_list)
            init_err_list = np.array(init_err_list)

            # TODO: checking cm algorithm to avoid coordinate normalization
            cm_value = CM.main(init_err_list, init_pos_list[:, 0], init_pos_list[:, 1])

            if max_cm < cm_value:
                max_cm = cm_value
                disassemble_idx = curr_idx

        if disassemble_idx == -1:
            print('cannot find controllable assembly sequence')
            return
        init_G.disassemble([disassemble_idx], init_G.coord[disassemble_idx])
        detaching_list.remove(disassemble_idx)
        print('disassembly: with target ' + str(disassemble_idx) + ', cm = ' + str(max_cm))

        if is_visualization == 'dynamic':
            transform_step[i * 2 + 1, :, :] = np.array(init_G.coord)
            transform_idx[i*2] = disassemble_idx
            edg = init_G.edge
            graph_step.append(np.array(edg))
        elif is_visualization == 'static':
            G_viz_static(init_G, max_cm, 'disassemble', disassemble_idx)

        '''assembly process'''
        max_cm = 0
        assemble_target_idx = -1
        assemble_target_pos = np.array([])

        # for assembly, select position (a, b) s.t. the cm value of new assembling units is maximized
        # assumption: only working quadrotor to be considered

        for j in range(len(final_available_pos_list)):
            curr_idx = final_available_pos_list[j]

            if not init_G.is_assembly_available(final_pos[curr_idx]):
                continue

            curr_assembled_pos.append(final_pos[curr_idx])
            curr_final_error_idx.append(1)
            cm_value = CM.main(np.array(curr_final_error_idx),
                               np.array(curr_assembled_pos)[:, 0],
                               np.array(curr_assembled_pos)[:, 1])
            curr_assembled_pos.pop(num - remaining_num - i)
            curr_final_error_idx.pop(num - remaining_num - i)
            if max_cm < cm_value:
                max_cm = cm_value
                assemble_target_pos = final_pos[curr_idx]
                assemble_target_idx = curr_idx

        init_G.assemble([disassemble_idx], init_G.coord[disassemble_idx], assemble_target_pos)
        final_available_pos_list.remove(assemble_target_idx)
        curr_assembled_pos.append(final_pos[assemble_target_idx])
        curr_final_error_idx.append(1)
        print('assembly: move to ' + str(assemble_target_idx) + ', cm = ' + str(max_cm))

        if is_visualization == 'dynamic':
            transform_step[i * 2 + 2, :, :] = np.array(init_G.coord)
            edg = init_G.edge
            # graph_step.append(np.array(edg))
            transform_idx[i * 2 + 1] = disassemble_idx
        elif is_visualization == 'static':
            G_viz_static(init_G, max_cm, 'assemble', assemble_target_pos / w)

    graph_step.append(np.array(init_G.edge))    # add final state into graph list

    if is_visualization == 'dynamic':
        G_viz_dynamic(init_failed_idx, transform_step, transform_idx, graph_step)

    return init_G


def regular_reconfiguration(init_G, final_pos, final_err_pos, minn_assembly_idx, is_visualization='static'):
    # TODO: finish dynamic visualization for regular cases
    return

