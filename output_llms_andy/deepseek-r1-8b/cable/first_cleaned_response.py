import pychrono as pc
import pychrono.viewer.ircos as irc


sim = pc.Simulation("beam_simulation")


physics = pc.Physics("beam_physics")
gravity = pc.G(0, -9.81)
physics.set_gravity_field([0, -9.81, 0])


beam = pc.Rod("beam", 5, pc.Material("ANCF", 0.1, 0.2, 0.1))  
beam.set_cross_section(pc.CircularCrossSection(0.01, 1000))
beam.set_beam_properties(pc.BeamProperties(pc.MomentOfInertia.Circular(0.01, 0.01**4)))
beam.set_allow_large_deformations(True)


hinge = pc.FixedSupport("hinge")
free_end = pc.FreeSupport("free_end")


beam.add_element(0)
beam.element(0).node(0).attach_to(hinge)
beam.element(0).node(1).attach_to(free_end)


nodes = []
for i in range(10):
    nodes.append(pc.Node(pc.Vec3(0, 0, 0) if i % 2 == 0 else pc.Vec3(0.5, 0, 0), 0, 0, 0))
beam.set_nodes(nodes)


forces = pc.Forces()
forces.add_force("free_end", pc.Vec3(0, -1, 0))
physics.add_forces(forces)


sim.set_physics(physics)
sim.set_timestep(0.001)


scene = pc.Scene()
camera = scene.add_camera("default")
renderer = scene.add_renderer("Irrlicht", antialias=True)


irc.setup_camera(camera)
irc.add_antialiasing(4)
irc.set_lighting(1, 1, 1, 0.5)


def render():
    global scene, camera, renderer
    scene.clear()
    renderer.begin()
    
    
    for i in range(10):
        node = nodes[i]
        irc.draw_sphere(node.pos, 0.05, pc.Color(1, 0, 0))
    
    
    for i in range(5):
        elem = beam.element(i)
        irc.draw_line(elem.node(0).pos, elem.node(1).pos, 1, 0.1, 1, 1)
    
    renderer.end()


while True:
    sim.update()
    physics.update()
    forces.update()
    render()
    pc.util.sleep(1e-4)