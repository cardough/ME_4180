import sympy
import numpy as np

# 1. Define Symbols
q1, q2 = sympy.symbols('q1 q2')
qd1, qd2 = sympy.symbols('qd1 qd2')
qdd1, qdd2 = sympy.symbols('qdd1 qdd2')
l1, m1, m2, g = sympy.symbols('l1 m1 m2 g')

# 2. Forward Kinematics (Modified DH)
# Joint 1: Revolute around Z, offset by l1
# Joint 2: Prismatic along X (after -pi/2 rotation)
# Position of Link 1 COM (assumed at joint for simplicity)
p1 = sympy.Matrix([0, 0, l1])

# Position of Link 2 COM (variable q2)
# Based on your MDH: alpha=-pi/2 means the new Z is the old -Y
p2 = sympy.Matrix([
    q2 * sympy.cos(q1),
    q2 * sympy.sin(q1),
    l1
])

# 3. Velocities
v1 = p1.jacobian(sympy.Matrix([q1, q2])) * sympy.Matrix([qd1, qd2])
v2 = p2.jacobian(sympy.Matrix([q1, q2])) * sympy.Matrix([qd1, qd2])

# 4. Energy Equations
# Kinetic Energy (K = 1/2 * m * v^2)
# Simplified: ignoring rotational inertia (I=0) as per your previous setup
K = 0.5 * m1 * v1.dot(v1) + 0.5 * m2 * v2.dot(v2)

# Potential Energy (P = m * g * height)
# Height is the Z-component
P = m1 * g * p1[2] + m2 * g * p2[2]

# Lagrangian (L = K - P)
L = K - P

# 5. Euler-Lagrange Equation
# tau_i = d/dt(dL/dqd_i) - dL/dq_i
state = sympy.Matrix([q1, q2])
state_dot = sympy.Matrix([qd1, qd2])
state_ddot = sympy.Matrix([qdd1, qdd2])

tau = []
for i in range(2):
    # dL/dqd_i
    dL_dqd = sympy.diff(L, state_dot[i])
    # d/dt (dL/dqd_i)
    d_dt_dL_dqd = sum([sympy.diff(dL_dqd, state[j]) * state_dot[j] for j in range(2)]) + \
                 sum([sympy.diff(dL_dqd, state_dot[j]) * state_ddot[j] for j in range(2)])
    # dL/dq_i
    dL_dq = sympy.diff(L, state[i])
    
    tau.append(sympy.simplify(d_dt_dL_dqd - dL_dq))

print("--- Symbolic Joint Forces/Torques ---")
print(f"Tau 1 (Revolute): {tau[0]}")
print(f"Tau 2 (Prismatic): {tau[1]}")