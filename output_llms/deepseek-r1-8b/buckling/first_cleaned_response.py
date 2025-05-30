import pychrono as pyct
import pychrono.gmsh as pygms
import pychrono.objc as pyo
import pychrono.vis as pyv
import pychrono.math as pymath


pyct.init()


sim = pyct.Simulation()
sim.set_timestep(1e-4)  
sim.set_solver('hires')  
sim.set Visualization(pyv.Irrlicht)  


matSteel = pyct.Material(name='Steel', density=7850, young_modulus=200000, poisson_ratio=0.2, loss_factor=0.01)


beam_length = 10  
beam_cross_section = pyct.Rectangle(name='Rect', length=0.1, width=0.1, height=0.01)
beam_density = 8000  
beam_young_modulus = 200000  
beam_poisson_ratio = 0.2  


support1 = pyct.RigidBody(name='Support1')
support1.set_mass(0)
support1.set_body_fixed(True)
support1.set_position(pyct.Vector3(0, 0, 0))

support2 = pyct.RigidBody(name='Support2')
support2.set_mass(0)
support2.set_body_fixed(True)
support2.set_position(pyct.Vector3(beam_length, 0, 0))


motor = pyct.RigidBody(name='Motor')
motor.set_mass(0)
motor.set_body_fixed(False)
motor.set_position(pyct.Vector3(beam_length/2, 0, 0))
motor.set_initial_rotation(0)  


beam = pyct.FEABeam(name='Beam', length=beam_length, cross_section=beam_cross_section, material=matSteel, density=beam_density)
beam.set_supports(support1, support2)
beam.set_nodes(50)  
beam.set_initial_shape()  


for i in range(beam.nodes):
    node = beam.nodes[i]
    node.set_mass(0.1)  
    node.set_body_fixed(False)


for i in range(beam.nodes):
    node = beam.nodes[i]
    node.add_constraint(node.prev, pyct.Constraint(type=pyct.Constraint.FIX_XY))
    node.add_constraint(node.next, pyct.Constraint(type=pyct.Constraint.FIX_XY))


def motor_function(sim, motor, time):
    
    motor.set_position(pyct.Vector3(motor.get_position().x, motor.get_position().y, time))
    motor.set_rotation(pyct.Vector3(0, 0, time*2))  


sim.add_body(motor)
sim.add_function('motor', motor_function)


viz = pyv.Irrlicht.create renderer()
viz.add_camera(pyct.Vector3(10, 10, 10), pyct.Vector3(0, 0, 0), pyct.Vector3(0, 0, 1))
viz.add_light(0, 0, 0, 1, 1, 1)
viz.add_light(beam_length, 0, 0, 1, 1, 1)


viz.add_mesh(beam, color=pyct.Color(1, 0, 0), wireframe=True)


sim.start()


def custom_motor_force(sim, body, time):
    
    angle = body.get_rotation().z
    force = pyct.Vector3(0, 0, 0)
    force.x = 10 * pymath.sin(angle)
    force.y = 10 * pymath.cos(angle)
    return force


sim.add_force('motor', custom_motor_force)


sim.run(1000)