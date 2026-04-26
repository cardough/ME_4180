import bdsim
from bdsim import BDSim
from bdsim.blockdiagram import BlockDiagram

sim: BDSim = bdsim.BDSim()
bd: BlockDiagram = sim.blockdiagram()

# 1. Define blocks
demand = bd.STEP(T=1, name='demand')
sum_block = bd.SUM('+-', name='sum')
controller = bd.GAIN(10, name='proportional_gain')
plant = bd.LTI_SISO(0.5, [2, 1], name='plant') # Transfer function
scope = bd.SCOPE(name='scope')

# 2. Connect blocks for feedback
bd.connect(demand, sum_block[0])       # Setpoint to positive sum input
bd.connect(sum_block, controller)     # Error to controller
bd.connect(controller, plant)        # Controller signal to plant
bd.connect(plant, sum_block[1])      # Plant output back to negative sum input (Feedback)
bd.connect(plant, scope)             # Plant output to scope for viewing

# 3. Compile and Run
bd.compile()
results = sim.run(bd, T=10)
sim.showgraph(bd)
