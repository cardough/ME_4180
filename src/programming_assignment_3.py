from sympy import symbols, simplify, Eq, solve
from sympy.physics.mechanics import dynamicsymbols
t = dynamicsymbols._t
from sympy.utilities.lambdify import lambdify

import numpy as np
import matplotlib.pyplot as plt

import roboticstoolbox as rtb

## Part a: Third-order polynomial
def third_order_polynomial(θ_o, θ_dot_o, θ_f, θ_dot_f, t_f):
    a_0 = θ_o
    a_1 = θ_dot_o
    a_2 = (3/(t_f**2))*(θ_f - θ_o) - (2/t_f)*θ_dot_o - (1/t_f)*θ_dot_f
    a_3 = -(2/(t_f**3))*(θ_f - θ_o) + (1/(t_f**2))*(θ_dot_f + θ_dot_o)
    
    return a_0 + a_1*t + a_2*(t**2) + a_3*(t**3)

# Part b: Fifth-order polynomial
def fifth_order_polynomial(θ_o, θ_dot_o, θ_dbl_dot_o, θ_f, θ_dot_f, θ_dbl_dot_f, t_f):
    a_0 = θ_o
    a_1 = θ_dot_o
    a_2 = θ_dbl_dot_o/2
    a_3 = (20*θ_f - 20*θ_o - (8*θ_dot_f + 12*θ_dot_o)*t_f - (3*θ_dbl_dot_o - θ_dbl_dot_f)*(t_f**2)) / (2*(t_f**3))
    a_4 = (30*θ_o - 30*θ_f + (14*θ_dot_f + 16*θ_dot_o)*t_f - (3*θ_dbl_dot_o - 2*θ_dbl_dot_f)*(t_f**2)) / (2*(t_f**4))
    a_5 = (12*θ_f - 12*θ_o - (6*θ_dot_f + 6*θ_dot_o)*t_f - (θ_dbl_dot_o - θ_dbl_dot_f)*(t_f**2)) / (2*(t_f**5))
    
    return a_0 + a_1*t + a_2*(t**2) + a_3*(t**3) + a_4*(t**4) + a_5*(t**5)


# Part c: Two third-order polynomials with via point
def two_cubics_with_via(θ_o, θ_dot_o, θ_v, θ_f, θ_dot_f, t_v, t_f):
    
    # generate two cubics with an unknown velocity at the via point
    θ_dot_v = symbols("θ_dot_v")
    first_cubic = third_order_polynomial(θ_o, θ_dot_o, θ_v, θ_dot_v, t_v)
    second_cubic = third_order_polynomial(θ_v, θ_dot_v, θ_f, θ_dot_f, t_f-t_v)

    # find acceleration at via point for both cubics
    accel_1 = first_cubic.diff(t, 2).subs(t, t_v)
    accel_2 = second_cubic.diff(t, 2).subs(t, 0)
    
    # set the equations equal and solve for the unknown velocity,
    # then substitute back into the original expressions
    accel = Eq(accel_1, accel_2)
    first_cubic = first_cubic.subs(θ_dot_v, solve(accel, θ_dot_v)[0])
    second_cubic = second_cubic.subs(θ_dot_v, solve(accel, θ_dot_v)[0])
    
    return first_cubic, second_cubic


if __name__ == "__main__":
    
    # given parameters for the motion of A. and B.
    θ_o = 120           # degrees
    θ_dot_o = 0         # degrees/s
    θ_dbl_dot_o = 0     # degrees/s^2
    θ_f = 60            # degrees
    θ_dot_f = 0         # degrees/s
    θ_dbl_dot_f = 0     # degrees/s^2
    t_f = 1             # s
    
    
    n_derivatives = 3
    derivs = ['Angle', 'Velocity', 'Acceleration', "Jerk"]
    t_series = np.linspace(0, t_f, 50)
    plt.rcParams.update({
        'axes.titlesize': 14,   # Title size
        'axes.labelsize': 8,    # X and Y label size
        'xtick.labelsize': 6,   # X-tick size
        'ytick.labelsize': 6,   # Y-tick size
        'legend.fontsize': 8    # Legend size
        })
    fig, ax = plt.subplots(n_derivatives + 1, 1, figsize=(8, 10))
    ax[0].set_title("Various Trajectories with Time Derivatives")
    ax[0].set_ylabel("Position (degrees)")
    ax[1].set_ylabel("Velocity (degrees/s)")
    ax[2].set_ylabel("Acceleration (degrees/s^2)")
    ax[3].set_ylabel("Jerk (degrees/s^3)")
    ax[3].set_xlabel("Time (s)")
    
    
    # Plot trajectories (A.) and (B.)
    for i in range(2):
        if i == 0:
            name = "A. Cubic Polynomial"
            path = third_order_polynomial(θ_o, θ_dot_o, θ_f, θ_dot_f, t_f)
        if i == 1:
            name = "B. Quintic Polynomial"
            path = fifth_order_polynomial(θ_o, θ_dot_o, θ_dbl_dot_o, θ_f, θ_dot_f, θ_dbl_dot_f, t_f)
        print(f"Equations for {name}:")
        for k in range(n_derivatives + 1):
            vals = lambdify(t, simplify(path), modules='numpy')(t_series)
            if isinstance(vals, (float, np.floating)):
                vals = np.full_like(t_series, vals)
            ax[k].plot(t_series, vals, label=name)
            
            print(f"    {derivs[k]} = {simplify(path)}")
            
            path = path.diff(t)
    
    # plot rtb jtraj for verification
    traj = rtb.jtraj(θ_o, θ_f, 50)
    vals = [traj.q, traj.qd, traj.qdd, np.gradient(traj.qdd, t_series, axis=0)]
    for k in range(n_derivatives + 1):
        ax[k].plot(t_series, vals[k], lw=4, zorder=-10,
                   color='g', alpha=0.2, label='Corke rtb jtraj')
    ax[0].legend(loc='upper right')    
    
    
    # update parameters for next trajectory (problem C)
    θ_o = 60            # degrees
    θ_dot_o = 0         # degrees/s
    
    θ_v = 120           # degrees
    t_v = 1             # s
    
    θ_f = 30            # degrees
    θ_dot_f = 0         # degrees/s
    t_f = 2             # s
    
    # Plot trajectory (C.)
    fig2, ax2 = plt.subplots(n_derivatives + 1, 1, figsize=(8, 10))
    ax2[0].set_title("Two Cubic Trajectories with Time Derivatives")
    ax2[0].set_ylabel("Position (degrees)")
    ax2[1].set_ylabel("Velocity (degrees/s)")
    ax2[2].set_ylabel("Acceleration (degrees/s^2)")
    ax2[3].set_ylabel("Jerk (degrees/s^3)")
    ax2[3].set_xlabel("Time (s)")
    
    
    p1, p2 = two_cubics_with_via(θ_o, θ_dot_o, θ_v, θ_f, θ_dot_f, t_v, t_f)
    paths = [p1, p2]
    
    print("Equations for C. Two Cubics with Via:")
    for k in range(n_derivatives + 1):
        values = []
        for i, path in enumerate(paths):
            vals = lambdify(t, simplify(path), modules='numpy')(t_series)
            if isinstance(vals, (float, np.floating)):
                vals = np.full_like(t_series, vals)
            values.append(vals)
            print(f'    {derivs[k]}, cubic #{i+1} = {simplify(path)}')
            paths[i] = path.diff(t)
        vals1 = values[0][0:-1]
        values = np.concatenate((vals1, values[1]))
        ax2[k].plot(np.linspace(0, t_f, 99), values, label='C. Two Cubics with Via')
        
    ax2[0].legend(loc='upper right')
    plt.show()