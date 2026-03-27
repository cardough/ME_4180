from sympy import symbols, Matrix, sin, cos, eye, simplify, pprint, pi

def d_h_link_transform(alpha, a, d, theta):

    # computes the T matrix for a single link in terms of
    # that link's joint parameters

    T = Matrix([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
        [0, 0, 0, 1]
    ])

    return T

def total_transform(D_H_table):

    # Finds all the joint transforms for a given table
    # and multiplies them together to find T_total

    T_tot = eye(4)
    for i in range(D_H_table.shape[0]):
        alpha, a, d, theta = D_H_table[i, :]
        T_tot *= d_h_link_transform(alpha, a, d, theta)

    return simplify(T_tot)
        


l_1, l_2, l_3, θ_1, θ_2, θ_3 = symbols('l_1, l_2, l_3, θ_1, θ_2, θ_3')

# total table including gripper frame
d_h_table_H = Matrix([
    [0, 0, 0, θ_1],
    [0, l_1, 0, θ_2],
    [0, l_2, 0, θ_3],
    [0, l_3, 0, 0]
])

# not including gripper frame
d_h_table_3 = Matrix([
    [0, 0, 0, θ_1],
    [0, l_1, 0, θ_2],
    [0, l_2, 0, θ_3]
])


T_total_H = total_transform(d_h_table_H)
T_total_3 = total_transform(d_h_table_3)


T_total_H = T_total_H.subs({l_1:4, l_2:3, l_3:2})
T_total_3 = T_total_3.subs({l_1:4, l_2:3, l_3:2})

print("Symbolic total tranform matrix:")
pprint(T_total_H)
print("Symbolic frame 3 tranform matrix:")
pprint(T_total_3)

""" Case 1 """

print('Case 1 total transform matrix (0_H_T): ')
pprint(T_total_H.subs({θ_1:0, θ_2:0, θ_3:0}).evalf())

print('Case 1 frame 3 transform matrix (0_3_T): ')
pprint(T_total_3.subs({θ_1:0, θ_2:0, θ_3:0}).evalf())

""" Case 2 """
print('')
print('Case 2 total transform matrix (0_H_T): ')
pprint(T_total_H.subs({θ_1:10 * pi/180, θ_2:20 * pi/180, θ_3:30 * pi/180}).evalf())

print('Case 2 frame 3 transform matrix (0_3_T): ')
pprint(T_total_3.subs({θ_1:10 * pi/180, θ_2:20 * pi/180, θ_3:30 * pi/180}).evalf())


""" Case 3 """
print('')
print('Case 3 total transform matrix (0_H_T): ')
pprint(T_total_H.subs({θ_1:90 * pi/180, θ_2:90 * pi/180, θ_3:90 * pi/180}).evalf())

print('Case 3 frame 3 transform matrix (0_3_T): ')
pprint(T_total_3.subs({θ_1:90 * pi/180, θ_2:90 * pi/180, θ_3:90 * pi/180}).evalf())

