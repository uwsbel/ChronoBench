import pychrono as pc
import pybullet as bullet
import numpy as np


pc.init()
bullet.init()


sim = pc.Simulation(pc.GRavity, pc.NoConstraint, pc.NoExternalForces)



material = pc.Material()
material.name = "Kirchhoff"
material.E = 1000000000000  
material.nu = 0.3           
material.density = 1000    
sim.add_material(material)



msh = pc.mshgrid()
msh.h_size = 0.01
msh.w_size = 0.01
msh.x0 = -0.5
msh.y0 = -0.5
msh.x1 = 0.5
msh.y1 = 0.5
grid = msh.create()


nodes = []
for i in range(50):
    for j in range(50):
        nodes.append(pc.Node(grid.x0 + i * msh.h_size, grid.y0 + j * msh.w_size, 0))


elements = []
for i in range(50):
    for j in range(50):
        elements.append(pc.ShellElement(nodes[i], nodes[j]))


midpoints = []
for i in range(50):
    for j in range(50):
        midpoints.append(pc.Node((nodes[i].x + nodes[j].x)/2, (nodes[i].y + nodes[j].y)/2, 0))


for i in range(50):
    for j in range(50):
        elements[i].attach(nodes[i], nodes[j])
        elements[i].attach(nodes[i], midpoints[i*50 + j])


tablecloth = pc.Shell(nodes, elements, midpoints, material, mass=0.01)
sim.add_body(tablecloth)


ground = pc.RigidBody()
ground.mass = 1000000  
ground.position = np.array([0, 0, 0])
sim.add_body(ground)


renderer = bullet.Renderer()
renderer.setSimulation(sim)
renderer.setBullet(True)
sim.add_renderer(renderer)


sim.set_solver(pc.PardisoMKL())


tablecloth.body.position = np.array([0.5, 0.5, 0.01])
tablecloth.body.rotation = np.array([0, 0, 0])




sim.start()


while True:
    sim.step()
    renderer.render()