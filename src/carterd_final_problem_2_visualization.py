import bdsim
from bdsim import BDSim
from bdsim.blockdiagram import BlockDiagram

import matplotlib.pyplot as plt

sim: BDSim = bdsim.BDSim()
bd: BlockDiagram = sim.blockdiagram()

# Provided variables from table
L = 0.0006 # H, armature inductance
R = 1.4 # Ohms, armature resistance
K_a = 12 # amplifier gain
K_b = 0.00867 # V/deg/s, back emf constant
J_m = 0.00844 # lbf-in-s^2, lumped motor polar inertia
C_m = 0.00013 # lbf-in/deg/s, motor shaft viscous damping coefficient
n = 200 # gear ratio
K_m = 4.375 # lbf-in/A, torque constant
J_L = 1 # lbf-in-s^2, lumped load polar inertia
C_L = 0.5 # lbf-in/deg/s, load shaft viscous damping coefficient
g = 0 # ignore gravity at first
K_e = 1 # encode transfer function

# add in load inertia and damping
J = J_m + J_L/(n**2)
C = C_m + C_L/(n**2)

# Open-loop electromechanical system
amp = bd.GAIN(K_a, name='Amp')
rl_circuit = bd.LTI_SISO([1.0], [L, R], name='RL Circuit')
motor_torque = bd.GAIN(K_m, name='K_a')
jc_motor_dynamics = bd.LTI_SISO([1.0], [J, C], name='JC Motor Dynamics')
back_emf = bd.GAIN(K_b, name='Back EMF')
gear_ratio = bd.GAIN(1/n, name='Gear Ratio')
integrator = bd.LTI_SISO([1.0], [1, 0], name='Integrator')

rl_circuit[0] = amp - back_emf
motor_torque[0] = rl_circuit
jc_motor_dynamics[0] = motor_torque
back_emf[0] = jc_motor_dynamics
gear_ratio[0] = jc_motor_dynamics
integrator[0] = gear_ratio

# Closed-loop feedback control
pid = bd.LTI_SISO([1.0], [1, 0], name='PID Controller')
commanded_theta_L = bd.STEP(T=0.0, on=60.0, off=0.0, name='Commanded Theta_L') # commanded input
encoder = bd.GAIN(K_e, name='Encoder')
scope = bd.SCOPE(styles=['k', 'r--'], name='Angle and Angular Velocity')
v_scope = bd.SCOPE(styles=['k', 'r--'], name='Control Effort and Back emf')

pid[0] = commanded_theta_L - encoder
amp[0] = pid
encoder[0] = integrator
scope[0] = integrator # angle
scope[1] = gear_ratio # angular velocity
v_scope[0] = amp # control effort voltage
v_scope[1] = back_emf # back emf

# Compile and Run
bd.compile()
results = sim.run(bd, T=1.5)
sim.showgraph(bd)
