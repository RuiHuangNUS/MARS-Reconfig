import numpy as np
from queue import Queue

dir_expression = np.array([[-1, 0],[0, 1],[1, 0],[0, -1]])
w = 0.53  # m
connection_distance_threshold = 0.01

class Quadrotor_graph():
    def __init__(self, node_num, state_position, error_position, w=0.53):
        """
        :param node_num: the number of nodes for quadrotor system
        connecting direction:
            0: ↑
            1: →
            2: ↓
            3: ←
        """
        self.n = node_num
        self.w = w
        self.edge = np.zeros([node_num, 4])     # only can be connected in 4 directions
        self.coord = np.zeros([node_num, 2])    # coordinate of node
        self.is_working = np.zeros([node_num])
        self.direction = np.zeros([node_num])

        if node_num <= 0:
            return

        for i in range(node_num):
            self.edge[i, :] = int(-1)
            self.is_working[i] = error_position[i]
            self.coord[i, :] = state_position[i]
        # self.coord[0] = state_position[0]
        for i in range(1, node_num):
            self.assemble([i], self.coord[i, :], state_position[i])


    def add_edge(self, from_n, to_n, direction):
        # add edge from -> to
        self.edge[from_n, direction] = to_n

    def assemble(self, obj, set_coord, target_coord):  # obj can be consisted of multiple nodes
        for idx in range(np.size(obj)):
            node = obj[idx]
            connected = False
            for i in range(4):  # Here let i represent as assembly direction of idx
                # TODO:
                check_coord = target_coord + dir_expression[i] * w
                occupied = self.coord - check_coord
                occupied = np.sqrt(occupied[:, 0]*occupied[:, 0] + occupied[:, 1]*occupied[:, 1])
                temp = np.where([abs(occupied) < connection_distance_threshold])[1]

                if np.size(temp):
                    # add edge
                    connected = True
                    temp = int(temp[0])
                    dir_occupied = (i + 2) % 4
                    self.add_edge(node, temp, i)
                    self.add_edge(temp, node, dir_occupied)

            # Update coordinate
            if not connected and np.size(obj) <= 1:
                print("Notice: the quadrotor %d is not connected to any assembly" % node)
            else:
                self.coord[node, :] = target_coord[:]

            # update target coordinate according to set coordinate
            if idx + 1 < np.size(obj):
                target_coord = target_coord + (set_coord[idx+1] - set_coord[idx])

    def is_assembly_available(self, target_pos):
        for i in range(4):
            adjacent_pos = target_pos + dir_expression[i] * w
            t = np.where((self.coord == (adjacent_pos[0], adjacent_pos[1])).all(axis=1))
            if np.size(t):
                return True
        return False

    def disassemble(self, obj, center_disassembly):
        pos_shift = np.zeros([2])
        disassembled = False
        cnt_separating = np.size(obj)
        for idx in range(cnt_separating):
            node = obj[idx]
            for i in range(4):
                if np.size(np.where(obj == self.edge[node, i])):
                    continue
                check_coord = self.coord[node] + dir_expression[i] * w
                occupied = np.where((self.coord == (check_coord[0], check_coord[1])).all(axis=1))

                if np.size(occupied):
                    # delete edge
                    occupied = int(occupied[0])
                    disassembled = True
                    dir_occupied = (i + 2) % 4
                    self.edge[node, i] = int(-1)
                    self.edge[occupied, dir_occupied] = int(-1)
                    pos_shift += dir_expression[dir_occupied]*2
            if not disassembled:
                print("Notice: the quadrotor %d has been detached" % node)
                # return

        # else:
        center = np.sum(self.coord)/self.n
        width = np.max(self.coord[obj, 1]) - np.min(self.coord[obj, 1])
        height = np.max(self.coord[obj, 0]) - np.min(self.coord[obj, 0])
        maxn = np.max(self.coord[:, 0])
        minn = np.min(self.coord[:, 0])
        if center_disassembly[0] >= center:
            self.coord[obj, 0] = 2 * w + height + (self.coord[obj, 0] - center_disassembly[0]) + maxn
        else:
            self.coord[obj, 0] = -2 * w - height + (self.coord[obj, 0] - center_disassembly[0]) + minn

        if center_disassembly[1] >= center:
            self.coord[obj, 1] = w * 2 + width + (self.coord[obj, 1] - center_disassembly[1]) + maxn
        else:
            self.coord[obj, 1] = -w * 2 - width + (self.coord[obj, 1] - center_disassembly[1]) + minn
    #     pos_shift = (self.coord[obj] - np.sum(self.coord, axis=0))/abs(self.coord[obj] - np.sum(self.coord, axis=0))
    #     self.coord[obj] = pos_shift * 2 * w + self.coord[obj]
