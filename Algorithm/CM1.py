import numpy as np
from scipy.linalg import null_space
from itertools import combinations
import time

# Global variables
g0 = 9.8  # m/s^2
Jx = 0.0113
Jy = 0.0121
Jz = 0.0206
w = 0.53  # m
m_unit = 0.925 # kg
umin = 0     # N
umax = 5.125 # N
d = 0.16975 # m
k_u = 0.1

def generate_position_lists(M, N):
    # Configuration for positions
    # x_position_list and y_position_list are initially defined here
    # 3x3
    # 07-08-09
    # 04-05-06
    # 01-02-03
    # x_position_list = np.array([-w, -w, -w, 0, 0, 0, w, w, w])
    # y_position_list = np.array([-w, 0, w, -w, 0, w, -w, 0, w])
    # 3x3
    # 04-05-06
    # 01-02-03
    # x_position_list = np.array([-0.5*w, 0, 0.5*w, -0.5*w, 0, 0.5*w])
    # y_position_list = np.array([-0.5*w, -0.5*w, -0.5*w, 0.5*w, 0.5*w, 0.5*w])
    # Create grid of positions centered around (0, 0)
    
    # linspace generates M or N equally spaced values in the range
    x_values = np.linspace(-(M-1)/2 * w, (M-1)/2 * w, M)
    y_values = np.linspace(-(N-1)/2 * w, (N-1)/2 * w, N)
    
    # Use meshgrid to create the full grid of positions
    x_grid, y_grid = np.meshgrid(x_values, y_values)
    
    # Flatten the grids to generate the position lists
    x_position_list = x_grid.flatten()
    y_position_list = y_grid.flatten()
    
    return x_position_list, y_position_list

def calculate_center_of_mass(x_position_list, y_position_list):
    """Calculate the center of mass for the given position lists"""
    x_center_of_mass = np.mean(x_position_list)
    y_center_of_mass = np.mean(y_position_list)
    return x_center_of_mass, y_center_of_mass

def adjust_relative_positions(x_position_list, y_position_list, x_center_of_mass, y_center_of_mass):
    """Adjust position lists relative to the center of mass"""
    x_position_list_adjusted = x_position_list - x_center_of_mass
    y_position_list_adjusted = y_position_list - y_center_of_mass
    return x_position_list_adjusted, y_position_list_adjusted

def J_combination(X_position_list, Y_position_list, n_unit, ma):
    Jx_combination = Jx * n_unit + ma * np.sum(Y_position_list**2)
    Jy_combination = Jy * n_unit + ma * np.sum(X_position_list**2)
    Jz_combination = Jz * n_unit + ma * (np.sum(Y_position_list**2) + np.sum(X_position_list**2))
    return Jx_combination, Jy_combination, Jz_combination

def Obtain_Bf_Tg(rotor_angle, rotor_dir, rotor_ku, rotor_d, Rotors, rotor_Yita, x_unit, y_unit, ma):
    sz = len(rotor_angle)
    bt = np.zeros(sz)
    bl = np.zeros(sz)
    bm = np.zeros(sz)
    bn = np.zeros(sz)
    
    for i in range(sz):
        bt[i] = 1 * rotor_Yita[i]  # lift
        bl[i] = rotor_d[i] * np.sin(np.deg2rad(rotor_angle[i]))  # roll torque
        bm[i] = rotor_d[i] * np.cos(np.deg2rad(rotor_angle[i]))  # pitch torque
        bn[i] = rotor_dir[i] * rotor_ku[i] * rotor_Yita[i]  # yaw torque

    bl = -(bl + y_unit * np.ones(sz)) * rotor_Yita
    bm = (bm + x_unit * np.ones(sz)) * rotor_Yita
    
    Bf = np.vstack([bt, bl, bm, bn])
    Tg = np.array([ma * g0, 0, 0, 0])
    
    return sz, Bf, Tg

def acai(Bf, fcmin, fcmax, Tg):
    sz = Bf.shape
    n = sz[0]
    m = sz[1]
    M = np.arange(1, m + 1)
    S1 = np.array(list(combinations(M, n - 1)))
    sm = S1.shape[0]
    fc = (fcmin + fcmax) / 2
    Fc = np.dot(Bf, fc)
    dmin = np.zeros(sm)
    
    for j in range(sm):
        choose = S1[j, :] - 1
        B_1j = Bf[:, choose]
        z_jk = (fcmax - fcmin) / 2
        z_jk = np.delete(z_jk, choose)
        kesai = null_space(B_1j.T)
        kesai = kesai[:, 0]
        kesai = kesai / np.linalg.norm(kesai)
        B_2j = np.delete(Bf, choose, axis=1)
        E = np.dot(kesai.T, B_2j)
        dmax = np.abs(E) @ z_jk
        temp = dmax - np.abs(np.dot(kesai.T, (Fc - Tg)))
        dmin[j] = temp
    
    if np.min(dmin) >= 0:
        degree = np.min(dmin)
    else:
        degree = -np.min(np.abs(dmin))
    
    return degree

def Calculate_CM(error_id, M, N):  
    # Number of units
    x_position_list, y_position_list = generate_position_lists(M, N)
    n_unit = len(x_position_list)
    
    # Calculate the center of mass
    x_center_of_mass, y_center_of_mass = calculate_center_of_mass(x_position_list, y_position_list)
    
    # Adjust positions to make the center of mass located at the center of the coordinate system
    x_position_list_adj, y_position_list_adj = adjust_relative_positions(x_position_list, y_position_list, x_center_of_mass, y_center_of_mass)
    
    # Mass of the multirotor helicopter
    ma = m_unit * n_unit  # kg
    
    # Moment of inertia (example quadrotor unit configurations)
    Jx_comb, Jy_comb, Jz_comb = J_combination(x_position_list_adj, y_position_list_adj, n_unit, ma)
    Jf = np.diag([-ma, Jx_comb, Jy_comb, Jz_comb])
    
    # Matrix A and B for system dynamics
    A = np.block([
        [np.zeros((4, 4)), np.eye(4)],
        [np.zeros((4, 8))]
    ])
    B = np.vstack([np.zeros((4, 4)), np.linalg.inv(Jf)])
    
    # Rotor and configuration parameters
    s2i = {'anticlockwise': 1, 'clockwise': -1}
    rotor_ku = np.array([k_u, k_u, k_u, k_u])
    rotor_d = np.array([d, d, d, d])
    Rotors = np.array([1, 2, 3, 4])
    
    sz = 0
    Bf = []

    for i in range(n_unit):
        if error_id[i] == 0:
            rotor_Yita = np.array([0, 0, 0, 0])
            rotor_angle = np.array([45, 135, 225, 315])
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        elif error_id[i] == 1:
            rotor_Yita = np.array([1, 1, 1, 1])
            rotor_angle = np.array([45, 135, 225, 315])
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        elif error_id[i] == 2:  #
            rotor_Yita = np.array([0, 0, 1, 1])
            rotor_angle = np.array([45, 135, 225, 315])
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        elif error_id[i] == 3:  # Rotate 90 degrees counterclockwise
            rotor_Yita = np.array([0, 0, 1, 1])
            rotor_angle = np.array([315, 45,135, 225]) 
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        elif error_id[i] == 4:  # Rotate 180 degrees counterclockwise
            rotor_Yita = np.array([0, 0, 1, 1])
            rotor_angle = np.array([225, 315, 45,135])
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        elif error_id[i] == 5:  # Rotate 270 degrees counterclockwise
            rotor_Yita = np.array([0, 0, 1, 1])
            rotor_angle = np.array([135, 225, 315, 45])
            rotor_dir = np.array([s2i['anticlockwise'], s2i['clockwise'], s2i['anticlockwise'], s2i['clockwise']])
        # Additional configurations can be added here as needed

        sz_temp, Bf_temp, Tg_temp = Obtain_Bf_Tg(rotor_angle, rotor_dir, rotor_ku, rotor_d, Rotors, rotor_Yita, x_position_list_adj[i], y_position_list_adj[i], ma)
        sz += sz_temp
        Bf.append(Bf_temp)
    
    Bf = np.hstack(Bf)  # Combine Bf parts into a single matrix
    
    # Controllability Test Procedures
    Cab = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B, A @ A @ A @ A @ B, A @ A @ A @ A @ A @ B, A @ A @ A @ A @ A @ A @ B, A @ A @ A @ A @ A @ A @ A @ B])
    n = np.linalg.matrix_rank(Cab)

    # Step 2. Compute the ACAI
    Uset_umin = umin * np.ones(sz)
    Uset_umax = umax * np.ones(sz)
    
    # Start timing
    start_time = time.time()
    
    # Compute the ACAI
    Fc = Bf @ (Uset_umin + Uset_umax) / 2
    Tg = np.array([ma * g0, 0, 0, 0])
    ACAI = acai(Bf, Uset_umin, Uset_umax, Tg)
    
    if -1e-10 < ACAI < 1e-10:
        ACAI = 0
    
    print("ACAI:", ACAI)
    
    # End timing
    end_time = time.time()
    total_time = end_time - start_time
    
    # Determine controllability
    if n < A.shape[0] or ACAI <= 0:
        print('Uncontrollable')
    else:
        print('Controllable')
    return ACAI

if __name__ == "__main__":
    # Example default error_id
    error_id = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    M=3
    N=2
    Calculate_CM(error_id,M,N)