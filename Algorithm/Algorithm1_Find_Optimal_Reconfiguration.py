import numpy as np
import matplotlib.pyplot as plt
import copy
import imageio
from CM import Calculate_CM

# 定义正方形机器人模块
class Robot:
    def __init__(self, x, y, faulty=False, color='pink', rotor_faults=None):
        self.x = x
        self.y = y
        self.faulty = faulty
        self.orientation = 90  # 初始旋转角度设为90度，朝上
        self.color = 'orange' if faulty else color
        self.rotor_faults = rotor_faults if rotor_faults else [False, False, False, False]  # 四个螺旋桨是否故障的标记

    def rotate(self, degrees):
        self.orientation = (self.orientation + degrees) % 360  # 顺时针旋转
        # 根据旋转的角度调整螺旋桨的故障状态位置
        steps = (degrees // 90) % 4
        self.rotor_faults = self.rotor_faults[-steps:] + self.rotor_faults[:-steps]  # 同步更新故障状态位置

    def get_state(self):
        if self.faulty and all(self.rotor_faults):  # 所有螺旋桨都故障
            return 0  # 完全故障
        elif not self.faulty and not any(self.rotor_faults):  # 完全正常，没有任何故障
            return 1  # 完全正常
        else:
            if self.orientation == 90:
                return 2  # 不旋转，保持初始状态
            elif self.orientation == 0:
                return 5  # 逆时针旋转270度
            elif self.orientation == 270:
                return 4  # 逆时针旋转180度
            elif self.orientation == 180:
                return 3  # 逆时针旋转90度
            else:
                return 1  # 默认为完全正常状态

# 更新后的目标函数，接受status_list参数
def calculate_configuration_difference(status_list=None,M=1,N=1):
    # 如果提供了status_list，按需计算目标函数
    if status_list is not None:
        print(status_list)
        error_id=np.array(status_list)
        my_CM = Calculate_CM(error_id,M,N)
        # 确保my_CM是一个有效的数值
        if my_CM is None:
            print("Error: Calculate_CM returned None.")
            return float('inf')  # 返回一个较大的数值表示不良配置
        return my_CM
    else:
        # 如果没有提供status_list，返回一个较大的数值
        print("Error: Status list is not provided.")
        return float('inf')

# 根据箭头方向计算旋转后的螺旋桨位置
def get_rotated_positions(orientation):
    # 定义四个位置，以箭头为基准（不变编号）
    base_positions = [(0.75, 0.75), (0.75, 0.25), (0.25, 0.25), (0.25, 0.75)]
    
    # 根据当前的orientation调整位置，使得编号与箭头方向的相对位置保持不变
    steps = (orientation // 90) % 4
    base_positions = base_positions[-steps:] + base_positions[:-steps]
    
    return base_positions

# 可视化机器人的构型，并保存为图像
def visualize_robots(robots, M, N, title="Robot Configuration", save_path=None, annotation_text=None, text_position=(0.5, -0.5)):
    plt.figure()
    ax = plt.gca()
    
    # 设置边距
    margin = 0.4

    # Add robot numbering based on 3x3 grid positions
    robot_numbering = np.arange(1, M * N + 1).reshape(N, M)
    robot_numbering = np.flipud(robot_numbering)  # Flip to match the desired order (1-9 from top to bottom)
    
    for robot in robots:
        # 绘制机器人外部正方形
        rect = plt.Rectangle((robot.x, robot.y), 1, 1, fill=True, color=robot.color)
        ax.add_patch(rect)

        # 获取旋转后的螺旋桨位置
        circle_positions = get_rotated_positions(robot.orientation)
        rotor_labels = ['4', '1', '2', '3']  # 螺旋桨编号保持不变

        # 在正方形内部绘制四个小圆圈来表示四旋翼
        circle_radius = 0.15
        for i, (cx, cy) in enumerate(circle_positions):
            # 计算当前螺旋桨的编号
            rotor_label_index = (i + (robot.orientation // 90) - 1) % 4  # 计算当前编号位置，确保与旋转后的编号一致

            # 如果编号对应的 rotor_faults 为 True，则颜色为红色
            circle_color = 'red' if robot.rotor_faults[rotor_label_index] else 'green'
            circle = plt.Circle((robot.x + cx, robot.y + cy), circle_radius, color=circle_color, fill=True)
            ax.add_patch(circle)
            
            # 绘制螺旋桨编号
            plt.text(robot.x + cx, robot.y + cy, rotor_labels[i], fontsize=8, ha='center', va='center', color='white')

        # Display robot numbering at the center of each square
        plt.text(robot.x + 0.5, robot.y + 0.1, f'{robot_numbering[robot.y, robot.x]}', 
                 fontsize=14, ha='center', va='center', color='black', weight='bold')

        # 绘制箭头表示方向，顺时针旋转
        arrow_length = 0.2
        if robot.orientation == 90:  # 朝上
            dx, dy = 0, arrow_length
        elif robot.orientation == 0:  # 朝右
            dx, dy = arrow_length, 0
        elif robot.orientation == 270:  # 朝下
            dx, dy = 0, -arrow_length
        elif robot.orientation == 180:  # 朝左
            dx, dy = -arrow_length, 0
        
        ax.arrow(robot.x + 0.5, robot.y + 0.5, dx, dy, head_width=0.2, head_length=0.2, fc='black', ec='white')

    # Turn off axis (remove ticks and labels)
    plt.axis('off')

    # 添加注释文本（如果有）
    if annotation_text:
            plt.text(text_position[0] * M, text_position[1] * N, annotation_text, 
                    fontsize=14, ha='right')

    # 调整图形显示范围，增加边距
    plt.xlim(-margin, M + margin)
    plt.ylim(-margin, N + margin)
    plt.title(title)
    plt.gca().set_aspect('equal', adjustable='box')

    if save_path:
        plt.savefig(save_path)
    plt.close()

# 生成初始构型
def generate_initial_configuration(M, N, non_symmetric_positions):
    robots = []
    for y in range(N):
        for x in range(M):
            color = 'pink' if (x, y) in non_symmetric_positions else 'grey'  # 非对称位置用粉色，对称位置用灰色
            robots.append(Robot(x=x, y=y, color=color))
    return robots

# 打印MxN网格状态
def print_grid_status(robots, M, N):
    status_grid = np.full((N, M), -1)  # 创建一个 MxN 的网格状态数组，初始值为 -1
    for robot in robots:
        if 0 <= robot.x < M and 0 <= robot.y < N:
            status_grid[robot.y, robot.x] = robot.get_state()  # 更新每个机器人位置的状态
    print("Current Grid Status:")
    print(status_grid)
    return status_grid.flatten().tolist()  # 返回平铺的状态列表

# 穷举法寻找故障机器人的最优位置和方向，并逐步绘制
def brute_force_optimal_configuration(original, faulty_robot, non_symmetric_positions, allow_rotation, M, N):
    best_configuration = None
    min_difference = float('inf')
    images = []  # 用于存储图像路径

    # Add robot numbering based on 3x3 grid positions
    robot_numbering = np.arange(1, M * N + 1).reshape(N, M)
    robot_numbering = np.flipud(robot_numbering)  # Flip to match the desired order (1-9 from top to bottom)

    for x, y in non_symmetric_positions:
        # 根据 allow_rotation 参数决定是否进行旋转
        rotation_angles = [0, 90, 180, 270] if allow_rotation else [0]

        for degrees in rotation_angles:
            new_configuration = copy.deepcopy(original)
            
            # 找到要替换的机器人位置并替换为故障机器人
            for idx, robot in enumerate(new_configuration):
                if robot.x == x and robot.y == y:
                    # 复制故障机器人，确保其不影响其他非故障机器人
                    current_faulty_robot = Robot(x=x, y=y, faulty=True, rotor_faults=faulty_robot.rotor_faults.copy())
                    current_faulty_robot.rotate(degrees)
                    new_configuration[idx] = current_faulty_robot

                    # 确保新的配置仍然是MxN网格
                    positions = [(r.x, r.y) for r in new_configuration]
                    if len(set(positions)) == len(new_configuration):  # 检查是否每个位置都是唯一的
                        # 打印当前MxN网格状态
                        status_list = print_grid_status(new_configuration, M, N)

                        # 使用自定义目标函数计算差异
                        difference = calculate_configuration_difference(status_list,M,N)
                        print(f"Difference for position ({x}, {y}) with rotation {degrees}° CM: {difference}")

                        # Get the robot number based on the current (x, y) coordinates
                        robot_number = robot_numbering[y, x]

                        # 绘制当前尝试的配置
                        img_path = f"step_{x}_{y}_{degrees}.png"
                        annotation_text = f"CM: {difference:.4f}"
                        # Inside brute_force_optimal_configuration
                        visualize_robots(new_configuration, M, N, 
                                        f"Step: Place Faulty Robot at No. {robot_number}, Orientation: {current_faulty_robot.orientation}°", 
                                        save_path=img_path, annotation_text=annotation_text, 
                                        text_position=(0.65, 1.05))  # Adjust the position as needed
                        images.append(img_path)

                        if difference < min_difference:
                            min_difference = difference
                            best_configuration = copy.deepcopy(new_configuration)

    # 生成GIF
    create_gif(images, 'robot_configuration_%sx%s.gif'%(M,N))
    return best_configuration


# 创建GIF
def create_gif(image_paths, output_path):
    images = []
    for path in image_paths:
        images.append(imageio.imread(path))
    imageio.mimsave(output_path, images, duration=0.5)
    print(f"GIF saved as {output_path}")

# 主函数
if __name__ == "__main__":
    M = 3  # 使用MxN的网格作为示例
    N = 3
    n = M * N  # 总的机器人数量
    # 设置哪个圈变红，表示螺旋桨故障（例如，设置第3个螺旋桨故障）
    rotor_faults = [True, True, True, True]  # 四个螺旋桨的故障状态 ['4', '1', '2', '3']
    
    # 定义非对称位置
    #3x3
    if M==3 and N==3:
        non_symmetric_positions = [(0, 1), (0, 2), (1, 1)]  # 例如，手动指定非对称位置
    #3x2
    if M==3 and N==2:
        non_symmetric_positions = [(0, 1), (1, 1)]  # 例如，手动指定非对称位置
    print("Non-symmetric positions for the faulty robot:", non_symmetric_positions)
    # 生成初始配置，标记非对称和对称位置
    original_robots = generate_initial_configuration(M, N, non_symmetric_positions)

    # 可视化初始构型
    visualize_robots(original_robots, M, N, "Initial Configuration")
    
    # 允许旋转或不允许旋转
    if all(rotor_faults):
        allow_rotation = False # 设置为 True 允许旋转，设置为 False 禁止旋转
    else:
        allow_rotation = True
    # 使用穷举法计算最优构型并逐步绘制
    faulty_robot = Robot(x=0, y=0, faulty=True, rotor_faults=rotor_faults)  # 故障机器人
    best_configuration = brute_force_optimal_configuration(original_robots, faulty_robot, non_symmetric_positions, allow_rotation, M, N)
    
    # 最终的最优构型
    visualize_robots(best_configuration, M, N, "Optimal Configuration with Faulty Robot")
