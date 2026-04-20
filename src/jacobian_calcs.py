from sympy import symbols, Matrix, zeros, expand, simplify, pi, diff
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.vector import vprint
import forward_kinematics

## velocity propagation code

def progagate_velocity(omega_base, v_base, T_link, θ_next_dot=0, d_next_dot=0):

    R_transpose = T_link[0:3, 0:3].T
    P_next = T_link[0:3, 3:4]
    Z_next = Matrix([0, 0, 1])

    omega_next = R_transpose * omega_base + θ_next_dot * Z_next

    v_next = R_transpose*(v_base + omega_base.cross(P_next)) + d_next_dot * Z_next

    return omega_next, v_next


def total_velocity(D_H_table):

    omega = Matrix([0, 0, 0])
    v = Matrix([0, 0, 0])

    for i in range(D_H_table.shape[0]):
        alpha, a, d, theta = D_H_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)

        theta_dot = diff(theta)
        d_dot = diff(d)

        omega, v = progagate_velocity(omega, v, T_link, theta_dot, d_dot)

    return simplify(omega), simplify(v)

def velocity_jacobian(dh_table, joint_params) -> Matrix:

    joint_velocities = diff(joint_params)
    
    _, v_end_frame = total_velocity(dh_table)
    v_end_frame = expand(v_end_frame)

    jacobian = zeros(3, joint_velocities.shape[0])

    for i, v_term in enumerate(v_end_frame):
        for k, joint_vel in enumerate(joint_velocities):
            jacobian[i, k] = v_term.coeff(joint_vel)
    
    return jacobian


## static force iteration code

# NOTE: this code does not work for prismatic joints
def force_jacobian(dh_table, joint_params):
    f_x, f_y, f_z = symbols('f_x, f_y, f_z')
    F = Matrix([f_x, f_y, f_z])
    net_torque = Matrix([0, 0, 0])
    joint_torques = zeros(joint_params.shape[0], 1)

    # iterate "backward" (end affector to base) through the d-h table,
    # storing the local torque about the z-axis for each joint
    for i in range(dh_table.shape[1] - 1, -1, -1):
        alpha, a, d, theta = dh_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)
        R_link = T_link[0:3, 0:3]
        P_link = T_link[0:3, 3:4]

        F = R_link*F
        net_torque = R_link*net_torque + P_link.cross(F)

        if i != 0:
            joint_torques[i-1] = net_torque[2]
        
    joint_torques = expand(joint_torques)

    jacobian_transpose = zeros(3, joint_params.shape[0])

    for i, v_term in enumerate(joint_torques):
        for k, tip_force in enumerate(Matrix([f_x, f_y, f_z])):
            jacobian_transpose[i, k] = v_term.coeff(tip_force)

    return simplify(jacobian_transpose.T)



if __name__ == "__main__":

    # setup robot arm
    l_1, l_2, l_3 = symbols('l_1, l_2, l_3')
    θ_1, θ_2, θ_3 = dynamicsymbols('θ_1, θ_2, θ_3')

    d_h_table_H = Matrix([
    [0,     0,      0,      θ_1],
    [pi/2,  l_1,    0,      θ_2],
    [0,     l_2,    0,      θ_3],
    [0,     l_3,    0,      0]
    ])

    joint_vars = Matrix([θ_1, θ_2, θ_3])

    # 1. find and print jacobian with velocity propagation
    print("Jacobian computed using velocity propagation: ")
    vprint(velocity_jacobian(d_h_table_H, joint_vars))
    print("")

    # 2. find and print jacobian with force iterations
    print("Jacobian computed using force iterations: ")
    vprint(force_jacobian(d_h_table_H, joint_vars))
