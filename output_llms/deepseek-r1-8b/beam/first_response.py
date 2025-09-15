import pychrono as pc
import pychrono.viewer as v
import numpy as np

# Initialize PyChrono and viewer
pc.init()
viewer = v.Viewer(window_size=(800, 600), enable_antialiasing=True)
viewer.add_light(ambient=True, color=(1,1,1), intensity=0.5)

# Create rigid body and mesh
body = pc.RigidBody("Beam Body", position=(0, 0, 0), mass=1)
beam_length = 5.0
beam_cross_section = 0.1  # Cross-sectional area
beam_young_modulus = 200  # Young's modulus
nodes = []
for i in range(0, beam_length, 0.1):
    nodes.append(pc.Node(f"Node {i}", position=(i, 0, 0)))
beam = pc.Beam(nodes[0], nodes[-1], cross_section=beam_cross_section, young_modulus=beam_young_modulus)

# Add nodes and beam to the scene
scene = pc.Scene()
scene.add_body(body)
scene.add_mesh(beam)
for node in nodes:
    scene.add_node(node)
spring = pc.Spring(nodes[0], 0.1, 100)  # Spring between first two nodes
scene.add_spring(spring)

# Add forces
force = pc.Forces()
force.add_spring(nodes[0], nodes[1], 0.1, 100)
body.add_forces(force)

# Visualization settings
viewer.add_scene(scene)
for node in nodes:
    node.mesh.color = (node.index / len(nodes), 0.5, 0.5)
scene.ambient_color = (0.2, 0.2, 0.2)

def simulate():
    viewer.clear_render()
    # Update simulation
    viewer.simulate()
    # Compute forces
    viewer.forces.update()
    # Render
    viewer.render()
    viewer.flip()

# Simulation loop
viewer.key_press_handler.add_handler(lambda: viewer.simulate(), " ")
viewer.show()