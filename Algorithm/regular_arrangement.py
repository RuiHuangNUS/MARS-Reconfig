import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import numpy as np
import CM
# import minn_controllability
from quadrotor_graph import Quadrotor_graph
import imageio

# N = 9
w = 0.53
ax_lim = 4
plt_time = 0.25
k=500
images = []
dir_expression = np.array([[-1, 0],[0, 1],[1, 0],[0, -1]])
visualization_type = {
    'none': 0,
    'static': 1,
    'dynamic': 2
}

def G_viz_static(G, cm_value, state, target_idx=[]):
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

        plt.annotate(str(i+1), xy=(G.coord[i, 1], -G.coord[i, 0]), xytext=(G.coord[i, 1], -G.coord[i, 0]+0.1))
    plt.show()


def G_viz_dynamic(err, transform_step, transform_idx, graph_step):
    plt.ion()
    step = (len(graph_step) - 2)*2
    frequency = 3
    for step_idx in range(step):
        current_transform_list = transform_idx[step_idx]
        if step_idx%2 == 0:     # for disassembly stage
            for num in range(frequency):
                plt.cla()
                cnt = step_idx
                sector = num + 1
                edge = graph_step[int(cnt / 2) + 1]

                for i in range(len(graph_step[0])):
                    if i in current_transform_list:
                        continue
                    for dir in range(4):
                        if edge[i, dir] > -1:
                            j = int(edge[i, dir])
                            xpoints = [transform_step[cnt, i, 1], transform_step[cnt, j, 1]]
                            ypoints = [-transform_step[cnt, i, 0], -transform_step[cnt, j, 0]]
                            plt.plot(xpoints, ypoints)

                for i in range(len(graph_step[0])):
                    if i in current_transform_list:
                        continue
                    if i == err:
                        plt.scatter(transform_step[cnt, i, 1], -transform_step[cnt, i, 0], s=w * k, c='r', marker='s')
                    else:
                        plt.scatter(transform_step[cnt, i, 1], -transform_step[cnt, i, 0], s=w * k, c='g', marker='s')
                    plt.annotate(str(i+1), xy=(transform_step[cnt, i, 1], -transform_step[cnt, i, 0]),
                                 xytext=(transform_step[cnt, i, 1], -transform_step[cnt, i, 0] + 0.1))

                moving_posi = transform_step[cnt, current_transform_list[:], :]
                next_moving_posi = transform_step[cnt+1, current_transform_list[:], :]
                center_posi = np.array([np.sum(moving_posi[:, 0]), np.sum(moving_posi[:, 1])])/len(moving_posi)
                next_center_posi = np.array([np.sum(next_moving_posi[:, 0]), np.sum(next_moving_posi[:, 1])])/len(moving_posi)
                delta = (next_center_posi - center_posi)
                moving_posi += delta / frequency * sector

                plt.scatter(moving_posi[:, 1], -moving_posi[:, 0], s=w * k, marker='s', c='g')

                for i_idx in range(len(current_transform_list)):
                    i = current_transform_list[i_idx]
                    plt.annotate(str(i+1), xy=(moving_posi[i_idx, 1], -moving_posi[i_idx, 0]),
                                 xytext=(moving_posi[i_idx, 1], -moving_posi[i_idx, 0] + 0.1))

                    for dir in range(4):
                        if edge[i, dir] > -1:
                            j = int(edge[i, dir])
                            j_idx = current_transform_list.index(j)
                            xpoints = [moving_posi[i_idx, 1], moving_posi[j_idx, 1]]
                            ypoints = [-moving_posi[i_idx, 0], -moving_posi[j_idx, 0]]
                            plt.plot(xpoints, ypoints)

                plt.xlim((-ax_lim, ax_lim))
                plt.ylim((-ax_lim, ax_lim))
                image_filename = f'disassembly_frame_{step_idx}_{num}.png'
                plt.savefig(image_filename)
                images.append(image_filename)
                plt.pause(plt_time)

        else:                       # for assembly stage
            # assembly trajectory is divided into 3 parts.
            moving = transform_idx[step_idx]

            start_pos = np.array([np.sum(transform_step[step_idx, moving, 0]), np.sum(transform_step[step_idx, moving, 1])])
            start_pos_center = start_pos / len(moving)
            final_pos = np.array([np.sum(transform_step[step_idx + 1, moving, 0]), np.sum(transform_step[step_idx + 1, moving, 1])])
            final_pos_center = final_pos/len(moving)

            relative_posi = transform_step[step_idx, moving, :] - start_pos_center

            # decide trajectory sequence using final_pos_center

            temp = start_pos_center
            target_pos = [np.array(start_pos_center)]

            rest_center = np.array([np.sum(transform_step[step_idx + 1, :, 0]), np.sum(transform_step[step_idx + 1, :, 1])])
            rest_center = (rest_center - final_pos)/(len(graph_step[0])-len(moving))
            print(rest_center)
            width = np.max(relative_posi[:, 1]) - np.min(relative_posi[:, 1])
            height = np.max(relative_posi[:, 0]) - np.min(relative_posi[:, 0])

            if rest_center[1] > final_pos_center[1]:
                temp[1] = final_pos_center[1] - 2 * w - width*1.5
                target_pos.append(np.array(temp))

            else:
                temp[1] = final_pos_center[1] + 2 * w + width*1.5
                target_pos.append(np.array(temp))


            if rest_center[0] > final_pos_center[0]:
                temp[0] = final_pos_center[0] - 2 * w + height*2
                target_pos.append(np.array(temp))
            else:
                temp[0] = final_pos_center[0] + 2 * w - height*2
                target_pos.append(np.array(temp))

            # if final_pos_center[1] > start_pos_center[1]:
            #     temp[1] = final_pos_center[1] + 2*w
            #     target_pos.append(np.array(temp))
            # else:
            #     temp[1] = final_pos_center[1] - 2*w
            #     target_pos.append(np.array(temp))

            # target_pos.append(np.array([final_pos_center[0], final_pos_center[1]]))

            target_pos.append(final_pos_center)

            for list_process in range(len(target_pos)-1):
                for num in range(frequency):
                    plt.cla()
                    cnt = list_process
                    sector = num + 1
                    edge = graph_step[int(step_idx / 2) + 1]

                    for i in range(len(graph_step[0])):
                        if i in current_transform_list:
                            continue
                        for dir in range(4):
                            if edge[i, dir] > -1:
                                j = int(edge[i, dir])
                                xpoints = [transform_step[step_idx, i, 1], transform_step[step_idx, j, 1]]
                                ypoints = [-transform_step[step_idx, i, 0], -transform_step[step_idx, j, 0]]
                                plt.plot(xpoints, ypoints)

                    for i in range(len(graph_step[0])):
                        if i in current_transform_list:
                            continue
                        if i == err:
                            plt.scatter(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0], s=w * k, c='r',
                                        marker='s')
                        else:
                            plt.scatter(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0], s=w * k, c='g',
                                        marker='s')
                        plt.annotate(str(i+1), xy=(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0]),
                                     xytext=(transform_step[step_idx, i, 1], -transform_step[step_idx, i, 0] + 0.1))

                    moving_posi = relative_posi
                    delta = (target_pos[cnt+1] - target_pos[cnt])
                    delta = delta / frequency * sector
                    moving_posi = moving_posi + delta + target_pos[cnt]


                    plt.scatter(moving_posi[:, 1], -moving_posi[:, 0], s=w * k, marker='s', c='g')

                    for i_idx in range(len(current_transform_list)):
                        i = current_transform_list[i_idx]
                        plt.annotate(str(i+1), xy=(moving_posi[i_idx, 1], -moving_posi[i_idx, 0]),
                                     xytext=(moving_posi[i_idx, 1], -moving_posi[i_idx, 0] + 0.1))

                        for dir in range(4):
                            if edge[i, dir] > -1:
                                j = int(edge[i, dir])
                                j_idx = current_transform_list.index(j)
                                xpoints = [moving_posi[i_idx, 1], moving_posi[j_idx, 1]]
                                ypoints = [-moving_posi[i_idx, 0], -moving_posi[j_idx, 0]]
                                plt.plot(xpoints, ypoints)

                    plt.xlim((-ax_lim, ax_lim))
                    plt.ylim((-ax_lim, ax_lim))
                    
                    #Gif
                    image_filename = f'assembly_frame_{step_idx}_{list_process}_{num}.png'
                    plt.savefig(image_filename)
                    images.append(image_filename)
                    plt.pause(plt_time)

    # final stage plotting
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
    gif_name='partial_self_reconfiguration.gif'
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

    IMPORTANT: no meaning of the array idx of final_pos and final_err_pos
    """
    if is_visualization == 'static':
        G_viz_static(init_G, CM.main(init_G.is_working, init_G.coord[:, 0], init_G.coord[:, 1]), 'initial')
    transform_step = np.zeros([100, len(final_err_pos), 2])
    transform_step[0, :, :] = np.array(init_G.coord)
    graph_step = [np.array(init_G.edge)]
    transform_idx = []
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
    '''step 2. calculate (dis)assembling sequence'''
    # for regular cases like M*N, at most 2 steps are taken to achieve self-reconfiguration

    x_sign = init_G.coord[init_failed_idx, 0] - np.sum(init_G.coord[:, 0])/num
    y_sign = init_G.coord[init_failed_idx, 1] - np.sum(init_G.coord[:, 1])/num

    starting_f = np.zeros([2])
    starting_i = np.zeros([2])
    temp_list = final_pos[final_available_pos_list]
    if x_sign < 0:
        maxn = np.max(final_pos[final_available_pos_list, 0])
        starting_f[0] = maxn
        temp_list = temp_list[np.where(temp_list[:, 0]==maxn)]
        starting_i[0] = np.max(init_G.coord[:, 0])
    else:
        minn = np.min(final_pos[final_available_pos_list, 0])
        starting_f[0] = minn
        temp_list = temp_list[np.where(temp_list[:, 0]==minn)]
        starting_i[0] = np.min(init_G.coord[:, 0])

    if y_sign < 0:
        starting_f[1] = np.max(temp_list[:, 1])
        starting_i[1] = np.max(init_G.coord[:, 1])
    else:
        starting_f[1] = np.min(temp_list[:, 1])
        starting_i[1] = np.min(init_G.coord[:, 1])

    starting_i = (np.where((init_G.coord==(starting_i[0], starting_i[1])).all(axis=1))[0])[0]
    starting_f = (np.where((final_pos == (starting_f[0], starting_f[1])).all(axis=1))[0])[0]


    for i in range(2):
        traverse_list.put(starting_i)
        traverse_list.put(starting_f)
        is_traverse[:] = 0

        disassemble = [starting_i]
        target_coord = final_pos[starting_f]
        # set_coord = init_G.coord[starting_i]

        while not traverse_list.empty():
            curr_idx = traverse_list.get()
            final_idx = traverse_list.get()
            is_traverse[curr_idx] = 1

            if not (curr_idx in detaching_list):
                continue

            final_available_pos_list.remove(final_idx)
            detaching_list.remove(curr_idx)

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

                    disassemble.append(to_idx)

        center = np.array([np.sum(init_G.coord[disassemble, 0]), np.sum(init_G.coord[disassemble, 1])])
        center = center/len(disassemble)

        init_G.disassemble(disassemble, center)
        if is_visualization == 'static':
            G_viz_static(init_G, 0, 'disassemble', disassemble)
        elif is_visualization == 'dynamic':
            transform_step[i * 2 + 1, :, :] = np.array(init_G.coord)
            transform_idx.append(disassemble)
            edg = init_G.edge
            graph_step.append(np.array(edg))

        set_coord = init_G.coord[disassemble]

        init_G.assemble(disassemble, set_coord, target_coord)
        if is_visualization == 'static':
            G_viz_static(init_G, 0, 'disassemble', disassemble)
        elif is_visualization == 'dynamic':
            transform_step[i * 2 + 2, :, :] = np.array(init_G.coord)
            transform_idx.append(disassemble)
            edg = init_G.edge
            # graph_step.append(np.array(edg))

        if not len(detaching_list):     # all units have been detached
            break
        else:
            # only need to select minimum coordinate
            minn = np.array([np.min(init_G.coord[detaching_list, 0]),
                             np.min(init_G.coord[detaching_list, 1])])
            starting_i = (np.where((init_G.coord == (minn[0], minn[1])).all(axis=1))[0])[0]

            minn = np.array([np.min(final_pos[final_available_pos_list, 0]),
                             np.min(final_pos[final_available_pos_list, 1])])
            starting_f = (np.where((final_pos == (minn[0], minn[1])).all(axis=1))[0])[0]


    #     if is_visualization:
    #         transform_step[i * 2 + 1, :, :] = np.array(init_G.coord)
    #         transform_idx[i*2] = disassemble_idx
    #         edg = init_G.edge
    #         graph_step.append(np.array(edg))
    if is_visualization == 'dynamic':
        edg = init_G.edge
        graph_step.append(np.array(edg))
        G_viz_dynamic(init_failed_idx, transform_step, transform_idx, graph_step)

    #     if is_visualization:
    #         transform_step[i * 2 + 2, :, :] = np.array(init_G.coord)
    #         edg = init_G.edge
    #         # graph_step.append(np.array(edg))
    #         transform_idx[i * 2 + 1] = disassemble_idx
    #
    # graph_step.append(np.array(init_G.edge))    # add final state into graph list


    return init_G

