import numpy as np
import matplotlib.pyplot as plt
import copy
import imageio
from CM1 import Calculate_CM

# Define square-shaped robot module
class Robot:
    def __init__(self, x, y, faulty=False, color='pink', rotor_faults=None):
        self.x = x
        self.y = y
        self.faulty = faulty
        self.orientation = 90  # Initial orientation set to 90 degrees, facing upwards
        self.color = 'orange' if faulty else color
        self.rotor_faults = rotor_faults if rotor_faults else [False, False, False, False]  # Fault status of four rotors

    def rotate(self, degrees):
        self.orientation = (self.orientation + degrees) % 360  # Clockwise rotation
        # Adjust rotor fault positions based on rotation angle
        steps = (degrees // 90) % 4
        self.rotor_faults = self.rotor_faults[-steps:] + self.rotor_faults[:-steps]  # Synchronize fault position updates

    def get_state(self):
        if self.faulty and all(self.rotor_faults):  # All rotors faulty
            return 0  # Completely faulty
        elif not self.faulty and not any(self.rotor_faults):  # Fully functional, no faults
            return 1  # Fully functional
        else:
            if self.orientation == 90:
                return 2  # No rotation, initial state
            elif self.orientation == 0:
                return 5  # Rotated 270 degrees counterclockwise
            elif self.orientation == 270:
                return 4  # Rotated 180 degrees counterclockwise
            elif self.orientation == 180:
                return 3  # Rotated 90 degrees counterclockwise
            else:
                return 1  # Default to fully functional state

# Updated objective function that accepts status_list parameter
def calculate_configuration_difference(status_list=None, M=1, N=1):
    # If status_list is provided, calculate the objective function accordingly
    if status_list is not None:
        print(status_list)
        error_id = np.array(status_list)
        my_CM = Calculate_CM(error_id, M, N)
        # Ensure my_CM is a valid value
        if my_CM is None:
            print("Error: Calculate_CM returned None.")
            return float('inf')  # Return a large value to indicate poor configuration
        return my_CM
    else:
        # If no status_list provided, return a large value
        print("Error: Status list is not provided.")
        return float('inf')

# Calculate rotated positions of rotors based on arrow orientation
def get_rotated_positions(orientation):
    # Define four positions based on the arrow's base (unchanging labels)
    base_positions = [(0.75, 0.75), (0.75, 0.25), (0.25, 0.25), (0.25, 0.75)]
    
    # Adjust positions based on current orientation to maintain relative position to arrow
    steps = (orientation // 90) % 4
    base_positions = base_positions[-steps:] + base_positions[:-steps]
    
    return base_positions

# Visualize robot configurations and save as image
def visualize_robots(robots, M, N, title="Robot Configuration", save_path=None, annotation_text=None, text_position=(0.5, -0.5)):
    plt.figure()
    ax = plt.gca()
    
    # Set margins
    margin = 0.4

    # Add robot numbering based on 3x3 grid positions
    robot_numbering = np.arange(1, M * N + 1).reshape(N, M)
    robot_numbering = np.flipud(robot_numbering)  # Flip to match desired order (1-9 from top to bottom)
    
    for robot in robots:
        # Draw robot's outer square
        rect = plt.Rectangle((robot.x, robot.y), 1, 1, fill=True, color=robot.color)
        ax.add_patch(rect)

        # Get rotated rotor positions
        circle_positions = get_rotated_positions(robot.orientation)
        rotor_labels = ['4', '1', '2', '3']  # Rotor labels remain the same

        # Draw four small circles inside the square to represent quadrotors
        circle_radius = 0.15
        for i, (cx, cy) in enumerate(circle_positions):
            # Calculate current rotor label
            rotor_label_index = (i + (robot.orientation // 90) - 1) % 4  # Calculate current label position to match rotated label

            # If rotor label corresponds to a faulty rotor, color it red
            circle_color = 'red' if robot.rotor_faults[rotor_label_index] else 'green'
            circle = plt.Circle((robot.x + cx, robot.y + cy), circle_radius, color=circle_color, fill=True)
            ax.add_patch(circle)
            
            # Draw rotor labels
            plt.text(robot.x + cx, robot.y + cy, rotor_labels[i], fontsize=8, ha='center', va='center', color='white')

        # Display robot numbering at the center of each square
        plt.text(robot.x + 0.5, robot.y + 0.1, f'{robot_numbering[robot.y, robot.x]}', 
                 fontsize=14, ha='center', va='center', color='black', weight='bold')

        # Draw arrow to indicate direction, clockwise rotation
        arrow_length = 0.2
        if robot.orientation == 90:  # Facing up
            dx, dy = 0, arrow_length
        elif robot.orientation == 0:  # Facing right
            dx, dy = arrow_length, 0
        elif robot.orientation == 270:  # Facing down
            dx, dy = 0, -arrow_length
        elif robot.orientation == 180:  # Facing left
            dx, dy = -arrow_length, 0
        
        ax.arrow(robot.x + 0.5, robot.y + 0.5, dx, dy, head_width=0.2, head_length=0.2, fc='black', ec='white')

    # Turn off axis (remove ticks and labels)
    plt.axis('off')

    # Add annotation text if any
    if annotation_text:
            plt.text(text_position[0] * M, text_position[1] * N, annotation_text, 
                    fontsize=14, ha='right')

    # Adjust display range to add margins
    plt.xlim(-margin, M + margin)
    plt.ylim(-margin, N + margin)
    plt.title(title)
    plt.gca().set_aspect('equal', adjustable='box')

    if save_path:
        plt.savefig(save_path)
    plt.close()

# Generate initial configuration
def generate_initial_configuration(M, N, non_symmetric_positions):
    robots = []
    for y in range(N):
        for x in range(M):
            color = 'pink' if (x, y) in non_symmetric_positions else 'grey'  # Non-symmetric positions in pink, symmetric in grey
            robots.append(Robot(x=x, y=y, color=color))
    return robots

# Print MxN grid status
def print_grid_status(robots, M, N):
    status_grid = np.full((N, M), -1)  # Create MxN grid status array, initial value -1
    for robot in robots:
        if 0 <= robot.x < M and 0 <= robot.y < N:
            status_grid[robot.y, robot.x] = robot.get_state()  # Update state of each robot's position
    print("Current Grid Status:")
    print(status_grid)
    return status_grid.flatten().tolist()  # Return flattened status list

# Brute force search for optimal position and orientation of faulty robot, visualize step by step
def brute_force_optimal_configuration(original, faulty_robot, non_symmetric_positions, allow_rotation, M, N):
    best_configuration = None
    min_difference = float('inf')
    images = []  # Store image paths

    # Add robot numbering based on 3x3 grid positions
    robot_numbering = np.arange(1, M * N + 1).reshape(N, M)
    robot_numbering = np.flipud(robot_numbering)  # Flip to match desired order (1-9 from top to bottom)

    for x, y in non_symmetric_positions:
        # Decide whether to rotate based on allow_rotation parameter
        rotation_angles = [0, 90, 180, 270] if allow_rotation else [0]

        for degrees in rotation_angles:
            new_configuration = copy.deepcopy(original)
            
            # Find the position to replace with the faulty robot
            for idx, robot in enumerate(new_configuration):
                if robot.x == x and robot.y == y:
                    # Copy faulty robot to ensure it does not affect other non-faulty robots
                    current_faulty_robot = Robot(x=x, y=y, faulty=True, rotor_faults=faulty_robot.rotor_faults.copy())
                    current_faulty_robot.rotate(degrees)
                    new_configuration[idx] = current_faulty_robot

                    # Ensure new configuration remains a MxN grid
                    positions = [(r.x, r.y) for r in new_configuration]
                    if len(set(positions)) == len(new_configuration):  # Check if all positions are unique
                        # Print current MxN grid status
                        status_list = print_grid_status(new_configuration, M, N)

                        # Use custom objective function to calculate difference
                        difference = calculate_configuration_difference(status_list, M, N)
                        print(f"Difference for position ({x}, {y}) with rotation {degrees}° CM: {difference}")

                        # Get the robot number based on the current (x, y) coordinates
                        robot_number = robot_numbering[y, x]

                        # Visualize the current trial configuration
                        img_path = f"step_{x}_{y}_{degrees}.png"
                        annotation_text = f"CM: {difference:.4f}"
                        # Inside brute_force_optimal_configuration
                        visualize_robots(new_configuration, M, N, 
                                        f"Step: Place Faulty Robot at No. {robot_number}, Orientation: {current_faulty_robot.orientation}°", 
                                        save_path=img_path, annotation_text=annotation_text, 
                                        text_position=(0.65, 1.05))  # Adjust position as needed
                        images.append(img_path)

                        if difference < min_difference:
                            min_difference = difference
                            best_configuration = copy.deepcopy(new_configuration)

    # Create GIF
    create_gif(images, 'robot_configuration_%sx%s.gif' % (M, N))
    return best_configuration


# Create GIF
def create_gif(image_paths, output_path):
    images = []
    for path in image_paths:
        images.append(imageio.imread(path))
    imageio.mimsave(output_path, images, duration=0.5)
    print(f"GIF saved as {output_path}")

# Main function
if __name__ == "__main__":
    # Use MxN grid as an example
    M = 3  
    N = 3
    n = M * N  # Total number of robots
    # Set which rotors are faulty (e.g., set the third rotor as faulty)
    rotor_faults = [True, True, True, True]  # Fault status of four rotors ['4', '1', '2', '3']
    
    # Define non-symmetric positions
    # 3x3
    if M == 3 and N == 3:
        non_symmetric_positions = [(0, 1), (0, 2), (1, 1)]  # Manually specify non-symmetric positions
    # 3x2
    if M == 3 and N == 2:
        non_symmetric_positions = [(0, 1), (1, 1)]  # Manually specify non-symmetric positions
    print("Non-symmetric positions for the faulty robot:", non_symmetric_positions)
    # Generate initial configuration, marking symmetric and non-symmetric positions
    original_robots = generate_initial_configuration(M, N, non_symmetric_positions)

    # Visualize initial configuration
    visualize_robots(original_robots, M, N, "Initial Configuration")
    
    # Allow or disallow rotation
    if all(rotor_faults):
        allow_rotation = False  # Set to True to allow rotation, False to disallow
    else:
        allow_rotation = True
    # Use brute force to calculate optimal configuration and visualize step by step
    faulty_robot = Robot(x=0, y=0, faulty=True, rotor_faults=rotor_faults)  # Faulty robot
    best_configuration = brute_force_optimal_configuration(original_robots, faulty_robot, non_symmetric_positions, allow_rotation, M, N)
    
    # Final optimal configuration
    visualize_robots(best_configuration, M, N, "Optimal Configuration with Faulty Robot")