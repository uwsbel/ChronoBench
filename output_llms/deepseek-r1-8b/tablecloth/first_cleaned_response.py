import pychrono as pyc
from pychrono import Material, ShellElement, Node, Element, Solver
from pychrono.chemistry import PardisoMKL
import numpy as np
import Irrlicht


pyc.initialize()
pyc.set_output_level(pyc.OutputLevel.NO_OUTPUT)


solver = Solver("PardisoMKL", max_iter=100000, tolerance=1e-8)
material = Material("Kirchhoff", E_young=10, nu_poisson=0.3, rho=1)



n_rows = 50
n_cols = 50
thickness = 0.1


nodes = []
for i in range(n_rows):
    for j in range(n_cols):
        nodes.append(Node(pyc.Vector3(float(j)/n_cols, float(i)/n_rows, 0.0)))


elements = []
for i in range(n_rows):
    for j in range(n_cols):
        
        elem = ShellElement(
            nodes[i*n_cols + j],
            nodes[(i-1)*n_cols + j] if i > 0 else nodes[0],
            nodes[i*n_cols + j],
            nodes[i*n_cols + (j-1)] if j > 0 else nodes[0],
            material=material,
            thickness=thickness
        )
        elements.append(elem)


system = pyc.System()
for elem in elements:
    system.add_element(elem)


for node in nodes:
    node.set_position(pyc.Vector3(float(j)/n_cols, float(i)/n_rows, 0.0))







irr = Irrlicht.create_scene()
camera = irr.add_camera("perspective")
renderer = irr.add_renderer("OpenGL")
light = irr.add_light("directional", position=(10, 10, 10))
system.set_visualization(irr.scene, renderer)


system.solver.solver.set_max_iter(100000)
system.solver.solver.set_tolerance(1e-8)


system.initialize()
current_time = 0.0

while True:
    if system.solver.solver.converged:
        break
    system.step()
    current_time += system.solver.solver.time_step
    if current_time > 1.0:
        break








pyc.finalize()