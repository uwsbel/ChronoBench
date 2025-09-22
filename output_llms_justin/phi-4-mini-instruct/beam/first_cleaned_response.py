```python
from pychrono import *
from pychrono.pbrt import *
from pychrono.physics import *
from pychrono.physics.solver import *
from pychrono.visualization import *


sys = ChronoSystem()


system = System(sys)


renderer = IrrlichtRenderer()
renderer.setScene(system)


mesh = Mesh()


node1 = Node()
node2 = Node()
beam = Beam(node1, node2)


node1.set_position(0, 0, 0)
node2.set_position(1, 0, 0)
beam.set_length(1)
beam.set_material(Material(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,0, 0