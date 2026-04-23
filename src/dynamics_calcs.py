from sympy import (symbols, Matrix, zeros, eye,
                   diff, simplify, pi, Expr, expand,
                   pretty_print)
from sympy.physics.mechanics import dynamicsymbols
t = dynamicsymbols._t
from sympy.physics.vector import vprint

import roboticstoolbox as rtb
from spatialmath.base import symbol

import forward_kinematics
import jacobian_calcs


## Iterative Newton-Euler Dynamics

def progagate_kinematics(
        T_link,         # link info
        omega_base, omega_dot_base, v_base, v_dot_base, #vels and accels to transmit
        θ_dot_next=0, θ_double_dot_next=0,              # joint parameters
        d_dot_next=0, d_double_dot_next=0):

    R_transpose = T_link[0:3, 0:3].T
    P_next = T_link[0:3, 3:4]
    Z_next = Matrix([0, 0, 1])
    P_c_next = Matrix([0, 0, 0]) # bro idk how to get this. 0s are just hardcoded man

    #propagate angular velocity and angular acceleration
    omega_next = R_transpose * omega_base + θ_dot_next * Z_next
    omega_dot_next = (R_transpose*omega_dot_base
                      + (R_transpose*omega_base).cross(θ_dot_next*Z_next)
                      + θ_double_dot_next*Z_next)
    
    #propagate linear velocity and linear acceleration
    v_next = R_transpose*(v_base + omega_base.cross(P_next)) + d_dot_next * Z_next
    
    v_dot_next = (R_transpose*(omega_dot_base.cross(P_next)
        + omega_base.cross(omega_base.cross(P_next))
        + v_dot_base)
        + 2*omega_next.cross(d_dot_next*Z_next) #account for prismatic joint
        + d_double_dot_next*Z_next)
    
    # find linear accel of the link centroid;
    # ts is only valid for rotational joints i think???
    v_dot_centroid = (omega_dot_next.cross(P_c_next)
                    + omega_dot_next.cross(omega_dot_next.cross(P_c_next))
                    + v_dot_next)

    return omega_next, omega_dot_next, v_next, v_dot_next, v_dot_centroid

def inward_forces(
        T_link, m_link, I_link, P_c_link,    # link info
        f_tip, n_tip,               # force/torque to transmit
        omega_link, omega_dot_link, v_dot_centroid):  # velocities and accels of link
    
    R = T_link[0:3, 0:3]
    P_next = T_link[0:3, 3:4]
    Z_next = Matrix([0, 0, 1])

    # newton's laws for given link w.r.t centriod
    F_net_centroid = m_link * v_dot_centroid
    N_net_centroid = (I_link*omega_dot_link
             + omega_link.cross(I_link*omega_link))
    
    # determine the support reactions at the base of the link
    f_base = R*f_tip + F_net_centroid
    n_base = (N_net_centroid
              + R*n_tip
              + P_c_link.cross(F_net_centroid)
              + P_next.cross(R*f_tip))

    return f_base, n_base




def iterative_newton_euler(D_H_table, link_masses, I_matrices):

    omegas = [Matrix([0, 0, 0])]
    omega_dots = [Matrix([0, 0, 0])]
    v = Matrix([0, 0, 0])
    v_dot = Matrix([0, 0, 0])
    v_dot_centroids = [Matrix([0, 0, 0])]


    # forward pass computing velocities and accels
    for i in range(D_H_table.shape[0]):
        alpha, a, d, theta = D_H_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)

        omega_next, omega_dot_next, v, v_dot, v_dot_centroid = progagate_kinematics(
            T_link,
            omegas[-1], omega_dots[-1], v, v_dot, 
            θ_dot_next=theta.diff(t), θ_double_dot_next=theta.diff(t, 2),
            d_dot_next=d.diff(t), d_double_dot_next=d.diff(t, 2))
        
        omegas.append(omega_next)
        omega_dots.append(omega_dot_next)
        v_dot_centroids.append(v_dot_centroid)

    
    joint_forces = [None for _ in range(D_H_table.shape[0] + 1)]
    joint_torques = [None for _ in range(D_H_table.shape[0] + 1)]

    joint_forces[-1] = Matrix([0, 0, 0])
    joint_torques[-1] = Matrix([0, 0, 0])

    # reverse pass computing forces
    for i in range(D_H_table.shape[0] - 1, -1, -1):
        #print(f'iteration {i}:')

        alpha, a, d, theta = D_H_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)

        P_c_link = Matrix([0, 0, 0])

        f_base, n_base = inward_forces(
            T_link, link_masses[i], I_matrices[i], P_c_link,    # link info
            joint_forces[i+1], joint_torques[i+1],               # force/torque to transmit
            omegas[i+1], omega_dots[i+1], v_dot_centroids[i+1])  # velocities and accels of link

        joint_forces[i] = f_base
        joint_torques[i] = n_base



    motor_torques = [simplify(tor[2]) for tor in joint_torques]
    motor_forces = [simplify(f[2]) for f in joint_forces]

    return motor_forces, motor_torques







# Lagrangian Code

def k_total(D_H_table, link_masses, link_Is, P_centroids):

    omega = Matrix([0, 0, 0])
    v = Matrix([0, 0, 0])

    k_total = 0.0

    # step forward through each link and add up k
    for i in range(D_H_table.shape[0]):

        #compute kinematics
        alpha, a, d, theta = D_H_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)

        R_transpose = T_link[0:3, 0:3].T
        P_next = T_link[0:3, 3:4]
        P_c = P_centroids[i]
        m = link_masses[i]
        I = link_Is[i]
        Z_next = Matrix([0, 0, 1])
        θ_dot_link = theta.diff(t)
        d_dot_link = d.diff(t)


        v = R_transpose*(v + omega.cross(P_next)) + d_dot_link * Z_next
        v_c = v + omega.cross(P_c)
        omega = R_transpose * omega + θ_dot_link * Z_next

        k_total += (0.5 * (m * v_c.T * v_c + omega.T * I * omega))[0]

    return k_total.simplify()

def u_total(D_H_table, link_masses, P_centroids):

    g = symbols('g')
    gravity = Matrix([0, 0, -g])

    T_total = eye(4)
    u_total = 0
    
    # step forward through each link and add up u
    for i in range(D_H_table.shape[0]):

        #compute kinematics
        alpha, a, d, theta = D_H_table[i, :]
        T_link = forward_kinematics.d_h_link_transform(alpha, a, d, theta)
        T_total *= T_link

        P_c_total = T_total * P_centroids[i].row_insert(P_centroids[i].rows, Matrix([1]))
        m = link_masses[i]
        

        u_total += -(m * gravity.T * P_c_total[0:3, :])[0]
    
    return u_total.simplify()

def lagrangian(D_H_table, link_masses, link_Is, P_centroids, joint_variables):
    k = k_total(D_H_table, link_masses, link_Is, P_centroids)
    u = u_total(D_H_table, link_masses, P_centroids)

    theta = joint_variables
    theta_dot = joint_variables.diff(t)

    tau = k.diff(theta_dot).diff(t) - k.diff(theta) + u.diff(theta)

    return tau.simplify()

def format_dynamic_eq(tau:Matrix, joint_params:Matrix):
    # Splits the joint force vector into inertia, coriolis/centrifugal, and gravity terms.
    # Returns them in a tuple; they must be "reconstituted" with (M*qdd + C + G) if
    # you want to use them as a dynamic equation
    
    theta = joint_params
    theta_dot = joint_params.diff(t)
    theta_double_dot = joint_params.diff(t, 2)
    
    n_joints = len(joint_params)
    
    grav_terms: Matrix = tau
    for i in range(n_joints):
        grav_terms = grav_terms.subs({theta_dot[i]: 0, theta_double_dot[i]: 0})
    G: Matrix = grav_terms
    
    M = Matrix((tau - G).diff(theta_double_dot).reshape(2,2))
    
    V = simplify(tau - M*theta_double_dot - G)
    
    # this part is witchcraft to factor out a velocity
    C = zeros(n_joints, n_joints)
    for k in range(n_joints):       # like why are there 3 loops? (holy O(n^2))
        for j in range(n_joints):
            for i in range(n_joints):
                # Calculate Christoffel symbol
                c_ijk = 0.5 * (diff(M[k, j], theta[i]) +
                            diff(M[k, i], theta[j]) - 
                            diff(M[i, j], theta[k]))
                C[k, j] += c_ijk * theta_dot[i]

    return M, V, G



# attempt at cartesian conversion of the dynamics
def cartesian_conversion(tau, dh_table:Matrix, joint_params:Matrix):
    theta_dot = joint_params.diff(t)
    x, y, z = dynamicsymbols("x, y, z")
    end_eff_pos = Matrix([x, y, z])
    end_eff_accel = end_eff_pos.diff(t, 2)
    J = jacobian_calcs.velocity_jacobian(dh_table, joint_params)
    J_dot = J.diff(t)
    J_inv = J.pinv()
    J_inv_T = J.T.pinv()
    
    M, V, G = format_dynamic_eq(tau, joint_params)
    
    M_x = J_inv_T*M*J_inv
    V_x = J_inv_T*M*J_inv*J_dot*theta_dot + J_inv_T*V
    G_x = J_inv_T*G
    
    return simplify(M_x*end_eff_accel + V_x + G_x)



if __name__ == "__main__":

    # setup robot arm
    l_1 = symbols('l_1')

    θ_1, d_1 = dynamicsymbols('θ_1, d_1')
    joint_vars = Matrix([θ_1, d_1])

    d_h_table_H = Matrix([
        [0,     0,      l_1,      θ_1],
        [-pi/2,  0,      d_1,      0],
    ])

    I_xx, I_yy, I_zz, m_1, m_2 = symbols('I_xx, I_yy, I_zz, m_1, m_2')

    Is = [zeros(3) for _ in range(d_h_table_H.shape[0])]
    Is[0] = Matrix([
        [I_xx,  0,      0],
        [0,     I_yy,   0],
        [0,     0,   I_zz],
    ])

    P_cs = [Matrix([0,   0,  -l_1/2]),
            Matrix([0,   0,  0])]



    # 1.a. Find joint torques/forces with Newton-Euler
    print("Joint torques found with iterative newton-euler: ")
    forces, torques = iterative_newton_euler(d_h_table_H, [m_1, m_2], Is)
    vprint(f't_1 = {torques[0]}')
    vprint(f'f_1 = {forces[1]}')
    print("")

    # 1.b. Find joint torques/forces with Lagrangian Method
    print("Joint torques found with Lagrangian: ")
    lagrangian_torques = lagrangian(d_h_table_H, [m_1, m_2], Is, P_cs, joint_vars)
    vprint(lagrangian_torques)
    print("")

    # 3. Convert dynamics equations to cartesian space
    print("Dynamic equation converted into cartesian forces in frame {2}:")
    cart = cartesian_conversion(lagrangian_torques, d_h_table_H, joint_vars)
    pretty_print(cart[0])
    print("")

